/**
 * @file speed_check.cpp
 * @brief SCDR算法性能测试工具
 * @author ERASOR团队
 * @description 该文件实现了SCDR算法的性能测试功能，用于评估算法在不同数据集上的运行速度
 */

// 标准库
#include <iostream>                             // 输入输出流
#include <vector>                               // 动态数组
#include <cstdlib>                              // 标准库函数
#include <ctime>                                // 时间处理
#include <algorithm>                            // 算法库
#include <string>                               // 字符串处理
#include <sstream>                              // 字符串流
#include <fstream>                              // 文件流
#include <time.h>                               // 时间函数
#include <termios.h>                            // 终端控制

// ROS相关库
#include <sensor_msgs/PointCloud2.h>           // ROS点云消息

// 自定义工具库
#include <unavlib/convt.h>                      // 数据转换工具
#include <unavlib/others.h>                     // 其他实用工具

// PCL点云处理库
#include <pcl_conversions/pcl_conversions.h>    // PCL与ROS消息转换
#include <pcl/conversions.h>                    // PCL数据转换
#include <pcl/io/pcd_io.h>                      // PCD文件读写
#include <pcl/registration/ndt.h>               // NDT配准算法
#include <pcl/filters/approximate_voxel_grid.h> // 近似体素网格滤波
#include <pcl/point_types.h>                    // 点云数据类型
#include <pcl/point_cloud.h>                    // 点云基础类
#include <pcl/PCLPointCloud2.h>                 // PCL点云消息
#include <pcl/visualization/pcl_visualizer.h>   // 点云可视化
#include <pcl/filters/passthrough.h>            // 直通滤波器
#include <pcl/common/transforms.h>              // 点云变换

// SCDR算法核心
#include <src/scdr.h>                           // SCDR算法实现

#define LINE_SPACE 10                           // 换行符定义

using namespace std;                            // 使用标准命名空间
using namespace unavlib;                        // 使用自定义命名空间

// 全局变量 - 数据路径配置
string root = "/media/shapelim/SAMSUNG/A4_XYZI";  // 数据根目录
string mapname = "/map/";                        // 地图数据子目录
string pc_currname = "/curr/";                   // 当前点云数据子目录
string posename = "/pose/";                      // 位姿数据子目录
string type_pcd = ".pcd";                        // PCD文件扩展名
string type_bin = ".bin";                        // 二进制文件扩展名

double max_d;                                    // 最大距离阈值（用于滤波）

/**
 * @brief 从二进制文件加载位姿矩阵
 * @param bin_name 二进制位姿文件路径
 * @return 4x4位姿变换矩阵
 * @description 读取KITTI格式的位姿文件，构建SE(3)变换矩阵
 */
Eigen::Matrix4f load_pose(string bin_name){
    std::ifstream in(bin_name,std::ios_base::binary);    // 以二进制模式打开文件
    Eigen::Matrix4f pose_se3;                           // 4x4位姿矩阵
    std::cout << "Loaded pose from " <<bin_name<< std::endl;
    float f = 0;                                         // 临时浮点数变量
    
    if(in.good())                                        // 检查文件是否成功打开
    {
      // 读取3x4的位姿矩阵（KITTI格式）
      for (int i = 0; i < 4; ++i){                      // 遍历列
        for (int j = 0; j < 3; ++j){                    // 遍历行（前3行）
            in.read((char *)&f,sizeof(float));          // 读取浮点数
            pose_se3(j, i) = f;                         // 存储到矩阵中
        }
      }
    }
    // 设置齐次坐标的最后一行 [0, 0, 0, 1]
    pose_se3(3, 0) = 0;
    pose_se3(3, 1) = 0;
    pose_se3(3, 2) = 0;
    pose_se3(3, 3) = 1;
    return pose_se3;                                     // 返回完整的4x4矩阵
}

/**
 * @brief 点云直通滤波函数
 * @param src 输入点云
 * @param dst 输出滤波后的点云
 * @description 对点云进行三维空间范围滤波，移除超出指定范围的点
 */
void pass_through(const pcl::PointCloud<pcl::PointXYZI>& src,
                  pcl::PointCloud<pcl::PointXYZI>& dst){
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> ptfilter;           // 直通滤波器

    *ptr_filtered = src;                                 // 复制输入点云

    // X轴方向滤波
    ptfilter.setInputCloud(ptr_filtered);                // 设置输入点云
    ptfilter.setFilterFieldName("x");                   // 设置滤波字段为x坐标
    ptfilter.setFilterLimits(-max_d, max_d);            // 设置x轴范围限制
    ptfilter.filter(*ptr_filtered);                     // 执行滤波

    // Y轴方向滤波
    ptfilter.setInputCloud(ptr_filtered);                // 更新输入点云
    ptfilter.setFilterFieldName("y");                   // 设置滤波字段为y坐标
    ptfilter.setFilterLimits(-max_d, max_d);            // 设置y轴范围限制
    ptfilter.filter(*ptr_filtered);                     // 执行滤波

    // Z轴方向滤波（高度限制）
    ptfilter.setInputCloud(ptr_filtered);                // 更新输入点云
    ptfilter.setFilterFieldName("z");                   // 设置滤波字段为z坐标
    ptfilter.setFilterLimits(-0.2, 2.0);               // 设置高度范围（-0.2m到2.0m）
    ptfilter.filter(*ptr_filtered);                     // 执行滤波

    dst = *ptr_filtered;                                 // 输出滤波结果
}

/**
 * @brief 加载PCD点云文件
 * @param pcd_name PCD文件路径
 * @param dst 输出点云数据
 * @return 成功返回0，失败返回-1
 * @description 从指定路径加载PCD格式的点云文件
 */
int load_pc(string pcd_name, pcl::PointCloud<pcl::PointXYZI>& dst){
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_pc(new pcl::PointCloud<pcl::PointXYZI>);
  
  // 尝试加载PCD文件
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_name, *ptr_pc) == -1)
  {
    PCL_ERROR ("Couldn't read file!!! \n");            // 输出错误信息
    return (-1);                                         // 返回错误码
  }
  
  // 输出加载成功信息
  std::cout << "Loaded " << ptr_pc->size () << " data points from " <<pcd_name<< std::endl;
  dst = *ptr_pc;                                         // 复制点云数据到输出
  return 0;                                              // 返回成功
}

/**
 * @brief 点云坐标变换函数
 * @param pose 4x4变换矩阵
 * @param src 输入点云
 * @param dst 输出变换后的点云
 * @description 使用给定的变换矩阵对点云进行坐标变换
 */
void transform(Eigen::Matrix4f& pose, const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst){
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(src, *ptr_transformed, pose); // 执行点云变换
  dst = *ptr_transformed;                                 // 输出变换结果
}

/**
 * @brief 显示交互式操作菜单
 * @description 在控制台显示可用的操作选项
 */
static void Display_Init_Menu()
{
  std::cout<<"----------- < Init menu > ----------"<<std::endl;
  std::cout<<"[r] Re-publish"<<std::endl;                    // 重新发布数据
  std::cout<<"[n] Next"<<std::endl;                          // 处理下一帧
  std::cout<<"[c] Kill the process"<<std::endl;              // 退出程序
  std::cout<<"----------------------------------------"<<std::endl;
}

/**
 * @brief 设置文件路径
 * @param filename 文件名（不含扩展名）
 * @param map_dir 输出地图文件路径
 * @param pc_dir 输出当前点云文件路径
 * @param pose_dir 输出位姿文件路径
 * @description 根据文件名构建完整的文件路径
 */
void set_dirs(const string& filename, string& map_dir, string& pc_dir, string& pose_dir){
  map_dir = root + mapname + filename + type_pcd;        // 构建地图文件路径
  pc_dir = root + pc_currname + filename + type_pcd;     // 构建当前点云文件路径
  pose_dir = root + posename + filename + type_bin;      // 构建位姿文件路径
}

/**
 * @brief 主函数 - SCDR算法性能测试程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 * @description 实现SCDR算法的性能测试，包括点云处理、算法执行时间统计等功能
 */
int main(int argc, char **argv){
  // ROS节点初始化
  ros::init(argc, argv, "main");                       // 初始化ROS节点
  ros::NodeHandle nh = ros::NodeHandle("~");            // 创建节点句柄
  
  // 参数声明
  int target_idx, num_rings, num_sectors;              // 目标索引、环数、扇区数
  double max_range, min_h, max_h;                       // 最大范围、最小高度、最大高度
  
  // 从ROS参数服务器获取参数
  nh.param("/idx", target_idx, 300);                   // 目标帧索引（默认300）
  nh.param("/scdr/max_range", max_range, 78.0);       // SCDR最大检测范围（默认78米）
  nh.param("/scdr/num_rings", num_rings, 20);         // SCDR环形分割数（默认20）
  nh.param("/scdr/num_sectors", num_sectors, 100);    // SCDR扇形分割数（默认100）
  nh.param("/scdr/max_h", max_h, 3.0);                // SCDR最大高度（默认3米）
  nh.param("/scdr/min_h", min_h, 0.0);                // SCDR最小高度（默认0米）
  
  // 输出参数配置信息
  cout<<"===========Setting params...============"<<endl;
  cout<<"target idx: "<< target_idx<<endl;             // 目标索引
  cout<<"max_range "<< max_range<<endl;               // 最大范围
  cout<<"num_rings "<< num_rings<<endl;               // 环数
  cout<<"num_sectors "<< num_sectors<<endl;           // 扇区数
  cout<<"max_h "<< max_h<<endl;                       // 最大高度
  cout<<"max_h "<< min_h<<endl;                       // 最小高度
  cout<<"========================================"<<endl;
  max_d = max_range;                                    // 设置全局最大距离

  // 创建ROS发布器
  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/map",100);           // 地图点云发布器
  ros::Publisher pub_pc_curr = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/pc2_curr",100);  // 当前点云发布器

  ros::Publisher pub_map_f = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/map_cut",100);     // 滤波后地图点云发布器
  ros::Publisher pub_pc_curr_f = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/pc2_curr_cut",100); // 滤波后当前点云发布器

  ros::Publisher pub_arranged = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/arranged",100);  // 排列后点云发布器
  ros::Publisher pub_complement = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/complement",100); // 补充点云发布器
  ros::Publisher pub_final = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/final",100);       // 最终结果点云发布器
  ros::Publisher pub_original = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/original",100); // 原始点云发布器
  ros::Rate loop_rate(5);                               // 设置循环频率为5Hz

  // 点云变量声明
  pcl::PointCloud<pcl::PointXYZI> pc_map, pc_curr;                    // 原始地图和当前点云
  pcl::PointCloud<pcl::PointXYZI> map_transformed, curr_transformed;  // 变换后的地图和当前点云
  pcl::PointCloud<pcl::PointXYZI> map_f, curr_f;                     // 滤波后的点云（filtered -> f）
  pcl::PointCloud<pcl::PointXYZI> map_arranged, map_complement;       // SCDR处理后的排列和补充点云
  pcl::PointCloud<pcl::PointXYZI> map_final, map_original, map_original_transformed; // 最终、原始和变换后的原始点云

  // 创建SCDR算法实例
  SCDR scdr = SCDR(max_range, num_rings, num_sectors, min_h, max_h);

  // 位姿变换矩阵声明
  Eigen::Matrix4f pose_curr, pose_prev, origin2body_curr, origin2body_prev, prev2curr;
  // ROS点云消息声明
  sensor_msgs::PointCloud2 pc2_map, pc2_map_original, pc2_curr, pc2_map_f, pc2_curr_f;
  // 文件路径变量
  string map_dir, pc_dir, pose_dir;

  // 数据加载和初始化
  string filename = std::to_string(target_idx);         // 将目标索引转换为文件名
  set_dirs(filename, map_dir, pc_dir, pose_dir);        // 设置文件路径

  pose_curr = load_pose(pose_dir);                       // 加载当前位姿
  origin2body_curr = pose_curr.inverse();               // 计算从原点到车体的变换矩阵

  // 加载点云数据
  load_pc(map_dir, pc_map);                             // 加载地图点云
  load_pc(pc_dir, pc_curr);                             // 加载当前点云
  map_original = pc_map;                                // 保存原始地图点云
  
  // 坐标变换
  transform(origin2body_curr, pc_map, map_transformed);              // 变换地图点云到车体坐标系
  transform(origin2body_curr, pc_curr, curr_transformed);            // 变换当前点云到车体坐标系
  transform(origin2body_curr, map_original, map_original_transformed); // 变换原始地图点云

  // 点云滤波
  pass_through(map_transformed, map_f);                 // 对变换后的地图点云进行滤波
  pass_through(curr_transformed, curr_f);               // 对变换后的当前点云进行滤波

  // 转换为ROS消息格式
  pc2_map = erasor_utilscloud2msg(map_transformed);     // 转换地图点云为ROS消息
  pc2_curr = erasor_utilscloud2msg(curr_transformed);   // 转换当前点云为ROS消息
  pc2_map_f = erasor_utilscloud2msg(map_f);             // 转换滤波后地图点云为ROS消息
  pc2_curr_f = erasor_utilscloud2msg(curr_f);           // 转换滤波后当前点云为ROS消息

  int count = 0;

  clock_t start, start2, end, end2;
  clock_t mid1, mid2, mid3;

  scdr.set_inputs(map_transformed, curr_transformed);
  scdr.compare_then_select();
  scdr.get_pcs(map_arranged, map_complement);
  map_final = map_arranged + map_complement;
  pose_prev = pose_curr;
  int tmp;
  while (true){
    if (tmp != LINE_SPACE){
      Display_Init_Menu();
    }
    std::cout<<"Press Key..."<<std::endl;

    tmp = getchar();

    if (tmp == LINE_SPACE) continue;
    cout<<"Debug: "<<tmp<<endl;
    switch (tmp){
      case 'n':
        count++;
        cout<<"Next"<<endl;
        target_idx += 5;
        filename = std::to_string(target_idx);
        set_dirs(filename, map_dir, pc_dir, pose_dir);

        pose_curr = load_pose(pose_dir);

        origin2body_curr = pose_curr.inverse();
        prev2curr = origin2body_curr * pose_prev;
        pc_map = map_final;

        load_pc(pc_dir, pc_curr);
        load_pc(map_dir, map_original);
        ROS_WARN_STREAM("On transforming...");

//        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
//        std::vector<int> pointIdxRadiusSearch;
//        std::vector<float> pointRadiusSquaredDistance;

        start = clock();
        transform(prev2curr, pc_map, map_transformed);
        mid1 = clock();
        transform(origin2body_curr, pc_curr, curr_transformed);
        end = clock();
        transform(origin2body_curr, map_original, map_original_transformed);

        mid2 = clock();
        pass_through(map_transformed, map_f);
        mid3 = clock();

        cout<<"tf: "<<(static_cast<double>(mid1 - start))/CLOCKS_PER_SEC <<std::endl;
        cout<<"filter: "<<(static_cast<double>(mid3 - mid2))/CLOCKS_PER_SEC <<std::endl;
        cout<<"tf: "<<(static_cast<double>(mid1 - start))/CLOCKS_PER_SEC/pc_map.size() * 10000 <<std::endl;
        cout<<"filter: "<<(static_cast<double>(mid3 - mid2))/CLOCKS_PER_SEC/pc_map.size() * 10000 <<std::endl;

        start2 = clock();
        pass_through(curr_transformed, curr_f);
        end2 = clock();
        cout<<"tf: "<<(static_cast<double>(end - mid1))/CLOCKS_PER_SEC <<std::endl;
        cout<<"filter: "<<(static_cast<double>(end2 - start2))/CLOCKS_PER_SEC <<std::endl;
        cout<<"tf: "<<(static_cast<double>(end - mid1))/CLOCKS_PER_SEC/pc_curr.size() * 10000 <<std::endl;
        cout<<"filter: "<<(static_cast<double>(end2 - start2))/CLOCKS_PER_SEC/pc_curr.size() * 10000 <<std::endl;


        pc2_map = erasor_utilscloud2msg(map_transformed);
        pc2_curr = erasor_utilscloud2msg(curr_transformed);
        pc2_map_f = erasor_utilscloud2msg(map_f);
        pc2_curr_f = erasor_utilscloud2msg(curr_f);
        scdr.set_inputs(map_transformed, curr_transformed);
        scdr.compare_then_select();
        scdr.get_pcs(map_arranged, map_complement);

        map_final = map_arranged + map_complement;
        if ((count + 1) % 7 == 0){
          double leaf_size = 0.25;
          static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
          pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>);
          pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
          *src = map_final;
          voxel_filter.setInputCloud(src);
          voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
          voxel_filter.filter(*ptr_voxelized);
          map_final = *ptr_voxelized;

        }
        ROS_WARN_STREAM("Removal done.");
        pose_prev = pose_curr;
        break;
      case 'p':
        cout<<"Republish"<<endl;

        break;
     case 'c':
       cout<<"Exit"<<endl;
       return 0;

     }

     for (int i=0; i < 3; ++i){
         if ( (i+1) % 2 == 0) cout<<i<<"th publish..."<<endl;


         start = clock();

//         cout<<i<<"th |";
//         cout<<"Takes "<<(static_cast<double>(end - start))/CLOCKS_PER_SEC <<std::endl;
//         cout<<(static_cast<double>(mid1 - start))/CLOCKS_PER_SEC<<", ";
//         cout<<(static_cast<double>(mid2 - mid1))/CLOCKS_PER_SEC<<", ";
//         cout<<(static_cast<double>(end - mid2))/CLOCKS_PER_SEC<<endl;

         sensor_msgs::PointCloud2 pc2_arranged = erasor_utilscloud2msg(map_arranged);
         sensor_msgs::PointCloud2 pc2_complement = erasor_utilscloud2msg(map_complement);
         sensor_msgs::PointCloud2 pc2_map_final = erasor_utilscloud2msg(map_final);
         sensor_msgs::PointCloud2 pc2_map_original = erasor_utilscloud2msg(map_original_transformed);

         pub_final.publish(pc2_map_final);
         pub_original.publish(pc2_map_original);
         pub_map.publish(pc2_map);
         pub_pc_curr.publish(pc2_curr);
         pub_map_f.publish(pc2_map_f); // for debugging
         pub_pc_curr_f.publish(pc2_curr_f);
         pub_arranged.publish(pc2_arranged);
         pub_complement.publish(pc2_complement);

         ros::spinOnce();
         loop_rate.sleep();
     }
  }
  return 0;
}


