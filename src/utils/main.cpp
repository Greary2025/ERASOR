/**
 * @file main.cpp
 * @brief SCDR算法主程序
 * @author ERASOR团队
 * @description 该文件实现了SCDR（Scan Context Descriptor Removal）算法的主程序，
 *              用于交互式地处理点云数据，进行动态对象移除和地图更新
 */

// 标准C++库
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>                               // 文件流操作
#include <time.h>
#include <termios.h>                             // 终端I/O控制

// ROS相关库
#include <sensor_msgs/PointCloud2.h>            // ROS点云消息类型

// PCL点云处理库
#include <pcl_conversions/pcl_conversions.h>     // PCL与ROS消息转换
#include <pcl/conversions.h>                     // PCL数据转换
#include <pcl/io/pcd_io.h>                       // PCD文件读写
#include <pcl/registration/ndt.h>                // NDT配准算法
#include <pcl/filters/approximate_voxel_grid.h>  // 近似体素网格滤波
#include <pcl/point_types.h>                     // 点云数据类型
#include <pcl/point_cloud.h>                     // 点云基础类
#include <pcl/PCLPointCloud2.h>                  // PCL点云消息类型
#include <pcl/visualization/pcl_visualizer.h>    // 点云可视化
#include <pcl/filters/passthrough.h>             // 直通滤波器
#include <pcl/common/transforms.h>               // 点云变换

// 自定义库
#include <unavlib/convt.h>                       // 转换工具
#include <unavlib/others.h>                      // 其他实用工具
#include <map_updater/src/scdr.h>                // SCDR算法

// 宏定义
#define LINE_SPACE 10                            // 行间距常量

using namespace std;
using namespace unavlib;

/**
 * @brief 数据路径配置
 * @description 定义了数据集的根目录和各种文件类型的子目录路径
 */
string root = "/media/shapelim/SAMSUNG/A4_XYZI";  // 数据集根目录
string mapname = "/map/";                         // 地图文件子目录
string pc_currname = "/curr/";                    // 当前点云文件子目录
string posename = "/pose/";                       // 位姿文件子目录
string type_pcd = ".pcd";                         // PCD文件扩展名
string type_bin = ".bin";                         // 二进制文件扩展名

/**
 * @brief 最大距离阈值
 * @description 用于点云滤波的最大距离参数
 */
double max_d;

/**
 * @brief 从二进制文件加载位姿矩阵
 * @param bin_name 二进制位姿文件路径
 * @return Eigen::Matrix4f 4x4位姿变换矩阵
 * @description 读取KITTI格式的位姿文件（3x4矩阵），并转换为4x4齐次变换矩阵
 */
Eigen::Matrix4f load_pose(string bin_name){
    std::ifstream in(bin_name,std::ios_base::binary);            // 以二进制模式打开文件
    Eigen::Matrix4f pose_se3;                                   // 创建4x4位姿矩阵
    std::cout << "Loaded pose from " <<bin_name<< std::endl;
    float f = 0;                                                 // 临时浮点数变量
    
    if(in.good())                                                // 检查文件是否成功打开
    {
      // 读取3x4位姿矩阵（KITTI格式）
      for (int i = 0; i < 4; ++i){                              // 遍历列
        for (int j = 0; j < 3; ++j){                            // 遍历行（只有前3行）
            in.read((char *)&f,sizeof(float));                  // 读取一个浮点数
            pose_se3(j, i) = f;                                 // 存储到矩阵中
        }
      }
    }
    
    // 补全齐次变换矩阵的最后一行 [0, 0, 0, 1]
    pose_se3(3, 0) = 0;
    pose_se3(3, 1) = 0;
    pose_se3(3, 2) = 0;
    pose_se3(3, 3) = 1;
    
    return pose_se3;                                             // 返回完整的4x4变换矩阵
}

/**
 * @brief 点云直通滤波器
 * @param src 输入点云
 * @param dst 输出点云
 * @description 对点云进行三维空间范围滤波，移除超出指定范围的点
 *              X和Y方向使用max_d作为范围，Z方向限制在-0.2到2.0米之间
 */
void pass_through(const pcl::PointCloud<pcl::PointXYZI>& src,
                  pcl::PointCloud<pcl::PointXYZI>& dst){
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> ptfilter;               // 创建直通滤波器

    *ptr_filtered = src;                                     // 复制输入点云

    // X方向滤波：保留[-max_d, max_d]范围内的点
    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("x");                       // 设置滤波字段为X坐标
    ptfilter.setFilterLimits(-max_d, max_d);                // 设置X方向范围
    ptfilter.filter(*ptr_filtered);

    // Y方向滤波：保留[-max_d, max_d]范围内的点
    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("y");                       // 设置滤波字段为Y坐标
    ptfilter.setFilterLimits(-max_d, max_d);                // 设置Y方向范围
    ptfilter.filter(*ptr_filtered);

    // Z方向滤波：保留[-0.2, 2.0]范围内的点（地面以上2米以内）
    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("z");                       // 设置滤波字段为Z坐标
    ptfilter.setFilterLimits(-0.2, 2.0);                    // 设置Z方向范围
    ptfilter.filter(*ptr_filtered);

    dst = *ptr_filtered;                                     // 输出滤波后的点云
}

/**
 * @brief 加载PCD点云文件
 * @param pcd_name PCD文件路径
 * @param dst 输出点云数据
 * @return int 加载状态（0表示成功，-1表示失败）
 * @description 从指定路径加载PCD格式的点云文件
 */
int load_pc(string pcd_name, pcl::PointCloud<pcl::PointXYZI>& dst){
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_pc(new pcl::PointCloud<pcl::PointXYZI>);
  
  // 尝试加载PCD文件
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_name, *ptr_pc) == -1)
  {
    PCL_ERROR ("Couldn't read file!!! \n");                 // 输出错误信息
    return (-1);                                             // 返回错误状态
  }
  
  // 输出加载成功信息
  std::cout << "Loaded " << ptr_pc->size () << " data points from " <<pcd_name<< std::endl;
  dst = *ptr_pc;                                             // 复制点云数据到输出
  return 0;                                                  // 返回成功状态
}

/**
 * @brief 点云坐标变换
 * @param pose 4x4变换矩阵
 * @param src 输入点云
 * @param dst 输出点云
 * @description 使用给定的变换矩阵对点云进行坐标变换
 */
void transform(Eigen::Matrix4f& pose, const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst){
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(src, *ptr_transformed, pose);    // 执行点云变换
  dst = *ptr_transformed;                                    // 复制变换后的点云
}

/**
 * @brief 显示交互式菜单
 * @description 在控制台显示可用的操作选项
 */
static void Display_Init_Menu()
{
  std::cout<<"----------- < Init menu > ----------"<<std::endl;
  std::cout<<"[r] Re-publish"<<std::endl;                     // 重新发布数据
  std::cout<<"[n] Next"<<std::endl;                           // 处理下一帧
  std::cout<<"[c] Kill the process"<<std::endl;               // 终止程序
  std::cout<<"----------------------------------------"<<std::endl;
}

/**
 * @brief 设置文件路径
 * @param filename 文件名（不含扩展名）
 * @param map_dir 地图文件完整路径（输出）
 * @param pc_dir 点云文件完整路径（输出）
 * @param pose_dir 位姿文件完整路径（输出）
 * @description 根据文件名构建完整的文件路径
 */
void set_dirs(const string& filename, string& map_dir, string& pc_dir, string& pose_dir){
  map_dir = root + mapname + filename + type_pcd;            // 构建地图文件路径
  pc_dir = root + pc_currname + filename + type_pcd;         // 构建当前点云文件路径
  pose_dir = root + posename + filename + type_bin;          // 构建位姿文件路径
}

/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态
 * @description SCDR算法的主程序入口，实现交互式点云处理和动态对象移除
 */
int main(int argc, char **argv){
  // 初始化ROS节点
  ros::init(argc, argv, "main");
  ros::NodeHandle nh = ros::NodeHandle("~");               // 创建私有节点句柄
  
  // 声明参数变量
  int init_idx, num_rings, num_sectors;                    // 初始索引、环数、扇区数
  double max_range, min_h, max_h;                          // 最大范围、最小高度、最大高度
  
  // 从ROS参数服务器获取参数
  nh.param("/idx", init_idx, 300);                        // 起始帧索引，默认300
  nh.param("/scdr/max_range", max_range, 78.0);          // SCDR最大检测范围，默认78米
  nh.param("/scdr/num_rings", num_rings, 20);            // SCDR环数，默认20
  nh.param("/scdr/num_sectors", num_sectors, 100);       // SCDR扇区数，默认100
  nh.param("/scdr/max_h", max_h, 3.0);                   // SCDR最大高度，默认3米
  nh.param("/scdr/min_h", min_h, 0.0);                   // SCDR最小高度，默认0米
  
  // 输出参数配置信息
  cout<<"===========Setting params...============"<<endl;
  cout<<"target idx: "<< init_idx<<endl;
  cout<<"max_range "<< max_range<<endl;
  cout<<"num_rings "<< num_rings<<endl;
  cout<<"num_sectors "<< num_sectors<<endl;
  cout<<"max_h "<< max_h<<endl;
  cout<<"min_h "<< min_h<<endl;                              // 修正输出标签
  cout<<"========================================"<<endl;
  
  max_d = max_range;                                       // 设置全局最大距离参数

  // 创建ROS发布器
  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/map",100);           // 地图点云发布器
  ros::Publisher pub_pc_curr = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/pc2_curr",100);  // 当前点云发布器

  ros::Publisher pub_map_f = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/map_cut",100);     // 滤波后地图点云发布器
  ros::Publisher pub_pc_curr_f = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/pc2_curr_cut",100); // 滤波后当前点云发布器

  ros::Publisher pub_arranged = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/arranged",100);  // 排列后点云发布器
  ros::Publisher pub_complement = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/complement",100); // 补充点云发布器
  ros::Publisher pub_final = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/final",100);       // 最终结果点云发布器
  ros::Publisher pub_original = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/original",100); // 原始点云发布器
  ros::Rate loop_rate(5);                                  // 设置循环频率为5Hz

  // 声明点云变量
  pcl::PointCloud<pcl::PointXYZI> pc_map, pc_curr;         // 地图点云和当前点云
  pcl::PointCloud<pcl::PointXYZI> map_transformed, curr_transformed; // 变换后的点云
  pcl::PointCloud<pcl::PointXYZI> map_f, curr_f;           // 滤波后的点云 (filtered -> f)
  pcl::PointCloud<pcl::PointXYZI> map_arranged, map_complement; // SCDR处理后的点云
  pcl::PointCloud<pcl::PointXYZI> map_final, map_original, map_original_transformed; // 最终结果和原始点云

  // 创建SCDR算法实例
  SCDR scdr = SCDR(max_range, num_rings, num_sectors, min_h, max_h);

  // 声明变换矩阵变量
  Eigen::Matrix4f pose_curr, pose_prev, origin2body_curr, origin2body_prev, prev2curr;
  // 声明ROS点云消息变量
  sensor_msgs::PointCloud2 pc2_map, pc2_map_original, pc2_curr, pc2_map_f, pc2_curr_f;
  // 声明文件路径变量
  string map_dir, pc_dir, pose_dir;

  // 初始化数据加载
  string filename = std::to_string(init_idx);              // 将索引转换为文件名
  set_dirs(filename, map_dir, pc_dir, pose_dir);           // 设置文件路径
  
  // 加载初始数据
  pose_curr = load_pose(pose_dir);                         // 加载当前位姿
  origin2body_curr = pose_curr.inverse();                 // 计算世界坐标到车体坐标的变换

  load_pc(map_dir, pc_map);                                // 加载地图点云
  load_pc(pc_dir, pc_curr);                                // 加载当前点云
  map_original = pc_map;                                   // 保存原始地图点云
  
  // 执行坐标变换（世界坐标系 -> 车体坐标系）
  transform(origin2body_curr, pc_map, map_transformed);    // 变换地图点云
  transform(origin2body_curr, pc_curr, curr_transformed);  // 变换当前点云
  transform(origin2body_curr, map_original, map_original_transformed); // 变换原始地图点云

  // 对变换后的点云进行滤波
  pass_through(map_transformed, map_f);                    // 滤波地图点云
  pass_through(curr_transformed, curr_f);                 // 滤波当前点云

  // 将PCL点云转换为ROS消息格式
  pc2_map = erasor_utilscloud2msg(map_transformed);       // 转换地图点云
  pc2_curr = erasor_utilscloud2msg(curr_transformed);     // 转换当前点云
  pc2_map_f = erasor_utilscloud2msg(map_f);               // 转换滤波后地图点云
  pc2_curr_f = erasor_utilscloud2msg(curr_f);             // 转换滤波后当前点云

  int count = 0;                                           // 处理帧计数器

  // 声明计时变量
  clock_t start, mid1, mid2, end;
  clock_t start_original, end_original;
  clock_t start_kd, end_kd;

  // 执行初始SCDR处理
  scdr.set_inputs(map_transformed, curr_transformed);     // 设置SCDR输入点云
  mid1 = clock();                                          // 记录开始时间
  scdr.compare_then_select();                             // 执行SCDR比较和选择
  mid2 = clock();                                          // 记录中间时间
  scdr.get_pcs(map_arranged, map_complement);             // 获取处理结果
  end = clock();                                           // 记录结束时间
  map_final = map_arranged + map_complement;              // 合并排列点云和补充点云
  pose_prev = pose_curr;                                   // 保存当前位姿作为前一帧位姿
  
  int tmp;                                                 // 用户输入字符变量
  
  // 主交互循环
  while (true){
    if (tmp != LINE_SPACE){                                // 如果不是空格键
      Display_Init_Menu();                                 // 显示操作菜单
    }
    std::cout<<"Press Key..."<<std::endl;                 // 提示用户按键

    tmp = getchar();                                       // 获取用户输入

    if (tmp == LINE_SPACE) continue;                       // 如果是空格键，继续循环
    cout<<"Debug: "<<tmp<<endl;                            // 输出调试信息
    
    // 根据用户输入执行相应操作
    switch (tmp){
      case 'n':                                            // 处理下一帧
        count++;                                             // 增加帧计数器
        cout<<"Next"<<endl;
        
        // 更新文件索引和路径
        init_idx += 5;                                       // 跳过5帧（可调整）
        filename = std::to_string(init_idx);                // 生成新文件名
        set_dirs(filename, map_dir, pc_dir, pose_dir);      // 设置新文件路径
        
        // 加载新的位姿和点云数据
        pose_curr = load_pose(pose_dir);                    // 加载当前帧位姿
        origin2body_curr = pose_curr.inverse();            // 计算新的坐标变换
        prev2curr = origin2body_curr * pose_prev;          // 计算前一帧到当前帧的变换
        pc_map = map_final;                                 // 使用上一帧的最终结果作为地图

        load_pc(pc_dir, pc_curr);                           // 加载当前帧点云
        load_pc(map_dir, map_original);                     // 加载原始地图点云
        ROS_WARN_STREAM("On transforming...");              // 输出变换提示

        // 执行坐标变换
        transform(prev2curr, pc_map, map_transformed);      // 变换地图点云（前一帧->当前帧）
        transform(origin2body_curr, pc_curr, curr_transformed); // 变换当前点云（世界->车体）
        transform(origin2body_curr, map_original, map_original_transformed); // 变换原始地图
        end_original = clock();                             // 记录变换结束时间

        // 对变换后的点云进行滤波
        pass_through(map_transformed, map_f);               // 滤波地图点云
        pass_through(curr_transformed, curr_f);             // 滤波当前点云

        // 转换为ROS消息格式
        pc2_map = erasor_utilscloud2msg(map_transformed);
        pc2_curr = erasor_utilscloud2msg(curr_transformed);
        pc2_map_f = erasor_utilscloud2msg(map_f);
        pc2_curr_f = erasor_utilscloud2msg(curr_f);

        // 执行SCDR算法
        scdr.set_inputs(map_transformed, curr_transformed); // 设置SCDR输入
        scdr.compare_then_select();                         // 执行比较和选择
        scdr.get_pcs(map_arranged, map_complement);         // 获取处理结果

        map_final = map_arranged + map_complement;          // 合并最终结果
        // 定期进行体素化下采样以减少点云密度
        if ((count + 1) % 7 == 0){                          // 每7帧执行一次下采样
          double leaf_size = 0.25;                          // 体素大小为0.25米
          static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter; // 静态体素滤波器
          pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>);
          pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
          *src = map_final;                                  // 复制最终地图
          voxel_filter.setInputCloud(src);                  // 设置输入点云
          voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size); // 设置体素大小
          voxel_filter.filter(*ptr_voxelized);              // 执行体素化滤波
          map_final = *ptr_voxelized;                       // 更新最终地图
        }
        
        ROS_WARN_STREAM("Removal done.");                   // 输出处理完成信息
        pose_prev = pose_curr;                              // 更新前一帧位姿
        break;
      case 'p':                                            // 重新发布数据
        cout<<"Republish"<<endl;
        break;
        
      case 'c':                                            // 退出程序
        cout<<"Exit"<<endl;
        return 0;

      default:                                             // 未知输入
        cout<<"Unknown command: "<<(char)tmp<<endl;
        break;
     }

     // 发布点云数据到ROS话题（连续发布3次以确保接收）
     for (int i=0; i < 3; ++i){
         if ( (i+1) % 2 == 0) cout<<i<<"th publish..."<<endl; // 输出发布进度

         // 性能计时代码（已注释）
         // cout<<i<<"th |";
         // cout<<"Takes "<<(static_cast<double>(end - start))/CLOCKS_PER_SEC <<std::endl;
         // cout<<(static_cast<double>(mid1 - start))/CLOCKS_PER_SEC<<", ";
         // cout<<(static_cast<double>(mid2 - mid1))/CLOCKS_PER_SEC<<", ";
         // cout<<(static_cast<double>(end - mid2))/CLOCKS_PER_SEC<<endl;

         // 转换处理结果为ROS消息格式
         sensor_msgs::PointCloud2 pc2_arranged = erasor_utilscloud2msg(map_arranged);     // 排列后点云
         sensor_msgs::PointCloud2 pc2_complement = erasor_utilscloud2msg(map_complement); // 补充点云
         sensor_msgs::PointCloud2 pc2_map_final = erasor_utilscloud2msg(map_final);       // 最终地图
         sensor_msgs::PointCloud2 pc2_map_original = erasor_utilscloud2msg(map_original_transformed); // 原始地图

         // 发布所有点云数据到相应话题
         pub_final.publish(pc2_map_final);          // 发布最终处理结果
         pub_original.publish(pc2_map_original);    // 发布原始地图
         pub_map.publish(pc2_map);                  // 发布变换后地图
         pub_pc_curr.publish(pc2_curr);             // 发布当前点云
         pub_map_f.publish(pc2_map_f);              // 发布滤波后地图（调试用）
         pub_pc_curr_f.publish(pc2_curr_f);         // 发布滤波后当前点云
         pub_arranged.publish(pc2_arranged);        // 发布排列后点云
         pub_complement.publish(pc2_complement);    // 发布补充点云

         ros::spinOnce();                           // 处理ROS回调
         loop_rate.sleep();                         // 按设定频率休眠
     }
  }
  return 0;                                                // 程序正常退出
}


