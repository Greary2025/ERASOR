/**
 * @file parse_data.cpp
 * @brief 数据解析和处理工具
 * @author ERASOR团队
 * @description 该文件实现了用于解析和处理SemanticKITTI数据集的工具，
 *              包括二进制文件读取、标签解析等功能
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
 * @brief 主程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态码
 * @description 该程序用于测试SemanticKITTI数据集的二进制文件读取功能，
 *              包括语义标签文件(.label)和点云文件(.bin)的读取测试
 */
int main(int argc, char **argv){

  // ==================== 测试方法1：逐步文件读取（已注释） ====================
  // 以下代码展示了如何逐步读取二进制标签文件的方法
  //  ifstream fin;                                                    ///< 文件输入流
  //  fin.open("/media/shapelim/UX960 NVMe/kitti_semantic/dataset/sequences/00/labels/000000.label", ios::binary);
  //  char buf[100000000];                                             ///< 缓冲区
  //  string s;                                                        ///< 字符串变量
  //  fin.seekg(0, ios::end);                                          ///< 移动文件指针到末尾
  //  int sz = fin.tellg();                                            ///< 获取文件大小
  //  std::cout<<sz<<std::endl;                                        ///< 输出文件大小
  //  fin.seekg(0, ios::beg);                                          ///< 移动文件指针到开头
  //  unsigned int f = 0;                                              ///< 临时变量
  //  fin.read((char *)&f, sizeof(int));                               ///< 读取一个整数
  //  cout<<f<<endl;                                                   ///< 输出读取的值

  // ==================== 测试方法2：批量文件读取 ====================
  
  // 读取语义标签文件测试
  std::ifstream input("/media/shapelim/UX960 NVMe/kitti_semantic/dataset/sequences/00/labels/000000.label", std::ios::binary);
  std::vector<uint32_t> buffer(std::istreambuf_iterator<char>(input), {});  ///< 将标签文件内容读入缓冲区
  std::cout << "Label file size: " << buffer.size() << " bytes" << std::endl;  ///< 输出标签文件大小

  // 读取点云文件测试
  std::ifstream input2("/media/shapelim/UX960 NVMe/kitti_semantic/dataset/sequences/00/veoldyne/000000.bin", std::ios::binary);
  std::vector<float> buffer2(std::istreambuf_iterator<char32_t>(input2), {}); ///< 将点云文件内容读入缓冲区
  std::cout << "Point cloud file size: " << buffer2.size() << " elements" << std::endl; ///< 输出点云文件大小

  return 0;                                                          ///< 程序正常退出
}


