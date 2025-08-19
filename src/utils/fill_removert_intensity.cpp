/**
 * @file fill_removert_intensity.cpp
 * @brief RemoveRT点云强度值填充工具
 * @author ERASOR团队
 * @description 该文件实现了一个工具，用于将RemoveRT算法处理后的点云与原始密集点云进行匹配，
 *              通过最近邻搜索为RemoveRT点云填充原始的强度值（包含语义标签信息）
 */

// ROS核心库
#include <ros/ros.h>

// PCL点云处理库
#include <pcl/common/transforms.h>           // 点云变换
#include <pcl_conversions/pcl_conversions.h> // PCL与ROS消息转换
#include <pcl/point_types.h>                 // 点云数据类型
#include <pcl/io/pcd_io.h>                   // PCD文件读写
#include <pcl/point_cloud.h>                 // 点云基础类
#include <pcl/filters/voxel_grid.h>          // 体素网格滤波器
#include <pcl/filters/passthrough.h>         // 直通滤波器
#include <pcl/kdtree/kdtree_flann.h>         // KD树最近邻搜索
#include <pcl/filters/extract_indices.h>     // 索引提取滤波器

// 自定义工具库
#include <unavlib/convt.h>                   // 转换工具
#include <unavlib/others.h>                  // 其他实用工具

// 标准库
#include <string>
#include <map>
#include <vector>

using namespace unavlib;

/**
 * @brief 动态对象类别ID列表
 * @description 定义了SemanticKITTI数据集中的动态对象类别标识符
 *              这些ID对应车辆、行人、自行车等可移动对象
 */
std::vector<int> dynamic_classes = {252, 253, 254, 255, 256, 257, 258, 259};


/**
 * @brief 为点云标记语义标签
 * @param src 源点云（需要标记的点云）
 * @param medium 参考点云（包含语义标签的密集点云）
 * @param dst 输出点云（标记后的点云）
 * @param leaf_size 体素化网格大小（米）
 * @description 该函数通过体素化和最近邻搜索，为源点云中的每个点
 *              分配来自参考点云的语义标签信息
 */
void label_map(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr medium,
                          pcl::PointCloud<pcl::PointXYZI>& dst,
                          double leaf_size){
  pcl::PointCloud<pcl::PointXYZI> tmp, output;
  int origin_size = (*src).points.size();                       // 记录原始点云大小
  
  // 第一步：对源点云进行体素化处理
  static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;           // 静态体素滤波器
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_filter.setInputCloud(src);                              // 设置输入点云
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);    // 设置体素网格大小
  voxel_filter.filter(*ptr_voxelized);                          // 执行体素化滤波
  tmp = *ptr_voxelized;

  // 输出体素化前后的点云大小变化（绿色文本）
  std::cout<<"\033[1;32m"<<origin_size<<" - > "<<tmp.points.size()<<"\033[0m"<<std::endl;

  // 第二步：通过最近邻搜索为每个点分配语义标签

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;                      // 创建KD树用于快速最近邻搜索
  kdtree.setInputCloud (medium);                                // 设置参考点云（包含语义标签）

  int K = 1;                                                     // 搜索最近的1个邻居
  std::vector<int> pointIdxNKNSearch(K);                        // 存储最近邻点的索引
  std::vector<float> pointNKNSquaredDistance(K);                // 存储最近邻点的距离平方
  
  // 遍历体素化后的每个点，为其分配语义标签
  for (const auto &pt: tmp.points){
    if ( kdtree.nearestKSearch (pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
          auto updated = pt;                                     // 复制当前点
          // 将参考点云中最近邻点的强度值（语义标签）赋给当前点
          updated.intensity = (*medium)[pointIdxNKNSearch[0]].intensity;
          output.points.push_back(updated);                      // 添加到输出点云
     }
  }
  dst = output;                                                  // 设置输出结果
}

/**
 * @brief 主函数 - RemoveRT强度值填充工具
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态码
 * @description 读取RemoveRT处理后的点云和原始密集点云，
 *              为RemoveRT点云填充语义标签信息并保存结果
 */
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "removert_update");
    std::cout<<"KiTTI MAPVIZ STARTED"<<std::endl;
    ros::NodeHandle nodeHandler;

    // 声明参数变量
    std::string mode, removertName, srcName, denseSrcName, seq;

    // 从ROS参数服务器获取配置参数
    nodeHandler.param<std::string>("/removert", removertName, "/media/shapelim");      // RemoveRT点云文件路径
    nodeHandler.param<std::string>("/DenseOriginal", denseSrcName, "/media/shapelim"); // 原始密集点云文件路径
    nodeHandler.param<std::string>("/seq", seq, "00");                                // 序列号
    nodeHandler.param<std::string>("/mode", mode, "rm3");                             // 处理模式

    ////////////////////////////////////////////////////////////////////
    // 数据加载和处理阶段
    ////////////////////////////////////////////////////////////////////

    std::cout<<"Loading map..."<<std::endl;
    std::cout<<removertName<<std::endl;                           // 输出RemoveRT文件路径
    std::cout<<denseSrcName<<std::endl;                           // 输出密集点云文件路径
    
    // 加载原始密集点云（包含语义标签）
    std::cout<<"\033[1;32mLoading dense map...\033[0m"<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_densemap(new pcl::PointCloud<pcl::PointXYZI>);
    datahandle3d::load_pcd(denseSrcName, ptr_densemap);           // 加载密集点云文件
    pcl::PointCloud<pcl::PointXYZI> srcDense;

    std::cout<<"Setting removert pcd..."<<std::endl;
    
    // 加载RemoveRT处理后的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_removert(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    datahandle3d::load_pcd(removertName, ptr_removert);           // 加载RemoveRT点云文件
    
    // 坐标变换矩阵（仅在需要时使用）
    Eigen::Matrix4f tf;
    // 注意：以下变换矩阵仅在RemoveRT算法需要时启用
    // 该变换将点云从一个坐标系转换到另一个坐标系
//    tf << 0,  0, 1, 0,      // 旋转和平移矩阵
//         -1,  0, 0, 0,
//          0, -1, 0, 1.73,
//          0,  0, 0, 1;
//    pcl::transformPointCloud(*ptr_removert, *ptr_transformed, tf);

    // 执行语义标签填充处理
    pcl::PointCloud<pcl::PointXYZI> removert_output;
    std::cout<<"Labeling removert pcd..."<<std::endl;
    label_map(ptr_removert, ptr_densemap, removert_output, 0.2);  // 体素大小设为0.2米
    
    // 保存处理后的点云
    std::cout<<"Saving removert pcd..."<<std::endl;
    removert_output.width = removert_output.points.size();        // 设置点云宽度
    removert_output.height = 1;                                   // 设置点云高度（无序点云）
    
    // 生成输出文件名（在原文件名后添加"_w_label"后缀）
    std::string tmp = removertName;
    tmp.erase(tmp.end()-4, tmp.end());                            // 移除".pcd"扩展名
    std::string save_name = tmp + "_w_label.pcd";                 // 添加标签后缀
    
    // 以ASCII格式保存点云文件
    pcl::io::savePCDFileASCII(save_name, removert_output);
    std::cout<<"\033[1;32mComplete to save!!!\033[0m"<<std::endl; // 绿色文本显示完成信息

    return 0;                                                     // 程序正常退出
}
