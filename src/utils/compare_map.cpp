/**
 * @file compare_map.cpp
 * @brief 地图比较和动态对象解析工具
 * @author ERASOR团队
 * @description 该文件实现了地图比较功能，包括动态对象解析、
 *              体素化处理和标签保留等核心功能
 */

// ROS相关库
#include <ros/ros.h>                                    ///< ROS核心功能

// PCL点云处理库
#include <pcl/common/transforms.h>                      ///< 点云变换
#include <pcl_conversions/pcl_conversions.h>            ///< PCL与ROS消息转换
#include <pcl/point_types.h>                            ///< 点云数据类型
#include <pcl/io/pcd_io.h>                              ///< PCD文件读写
#include <pcl/point_cloud.h>                            ///< 点云基础类
#include <pcl/common/transforms.h>                      ///< 点云变换（重复包含）
#include <pcl/filters/voxel_grid.h>                     ///< 体素网格滤波
#include <pcl/filters/passthrough.h>                    ///< 直通滤波器
#include <pcl/kdtree/kdtree_flann.h>                    ///< KD树快速最近邻搜索
#include <pcl/filters/extract_indices.h>                ///< 索引提取滤波器
#include <pcl/io/pcd_io.h>                              ///< PCD文件读写（重复包含）

// 自定义库
#include <unavlib/convt.h>                              ///< 转换工具
#include <unavlib/others.h>                             ///< 其他实用工具

// 标准C++库
#include <string>                                       ///< 字符串处理
#include <map>                                          ///< 映射容器
#include <vector>                                       ///< 动态数组

using namespace unavlib;

/// 动态对象类别标签列表（语义分割标签）
std::vector<int> dynamic_classes = {252, 253, 254, 255, 256, 257, 258, 259};

/**
 * @brief 解析动态对象和静态对象
 * @param cloudIn 输入点云（包含语义和实例标签）
 * @param dynamicOut 输出的动态对象点云
 * @param staticOut 输出的静态对象点云
 * @description 根据点云强度值中编码的语义标签，将点云分离为动态和静态两部分
 *              强度值编码格式：低16位为语义标签，高16位为实例标签
 */
void parse_dynamic_obj(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, pcl::PointCloud<pcl::PointXYZI>& dynamicOut, pcl::PointCloud<pcl::PointXYZI>& staticOut){
  dynamicOut.points.clear();                             ///< 清空动态对象点云容器
  staticOut.points.clear();                              ///< 清空静态对象点云容器

  for (const auto &pt: cloudIn.points){                  ///< 遍历输入点云中的每个点
    uint32_t float2int = static_cast<uint32_t>(pt.intensity); ///< 将强度值转换为32位整数
    uint32_t semantic_label = float2int & 0xFFFF;        ///< 提取低16位作为语义标签
    uint32_t inst_label = float2int >> 16;               ///< 提取高16位作为实例标签
    bool is_static = true;                               ///< 默认假设为静态对象
    
    // 检查是否属于动态对象类别
    for (int class_num: dynamic_classes){                ///< 遍历动态类别列表
      if (semantic_label == class_num){                  ///< 如果语义标签匹配动态类别
        dynamicOut.points.push_back(pt);                 ///< 添加到动态对象点云
        is_static = false;                               ///< 标记为动态对象
        break;                                           ///< 跳出循环
      }
    }
    
    if (is_static){                                      ///< 如果是静态对象
      staticOut.points.push_back(pt);                   ///< 添加到静态对象点云
    }
  }
}
/**
 * @brief 保留标签的体素化处理
 * @param src 输入点云智能指针
 * @param dst 输出体素化后的点云
 * @param leaf_size 体素网格叶子节点大小
 * @description 对点云进行体素化下采样，同时保留原始点的强度标签信息
 *              通过KD树最近邻搜索将原始标签分配给体素化后的点
 */
void erasor_utils::voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
                          pcl::PointCloud<pcl::PointXYZI>& dst,
                          double leaf_size){
  pcl::PointCloud<pcl::PointXYZI> tmp, output;           ///< 临时点云和输出点云容器

  // ==================== 第一步：体素化下采样 ====================
  static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;   ///< 静态体素网格滤波器
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>); ///< 体素化结果容器
  voxel_filter.setInputCloud(src);                      ///< 设置输入点云
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size); ///< 设置体素大小
  voxel_filter.filter(*ptr_voxelized);                  ///< 执行体素化滤波
  tmp = *ptr_voxelized;                                  ///< 复制体素化结果

  // ==================== 第二步：标签恢复 ====================
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;              ///< KD树用于最近邻搜索
  kdtree.setInputCloud(src);                            ///< 设置原始点云作为搜索树

  int K = 1;                                             ///< 搜索最近的1个点
  std::vector<int> pointIdxNKNSearch(K);                ///< 最近邻点索引容器
  std::vector<float> pointNKNSquaredDistance(K);        ///< 最近邻距离平方容器
  
  // 为每个体素化后的点恢复原始标签
  for (const auto &pt: tmp.points){                     ///< 遍历体素化后的每个点
    if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) ///< 执行最近邻搜索
    {
          auto updated = pt;                             ///< 复制当前点
          updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity; ///< 用最近邻点的强度值更新标签
          output.points.push_back(updated);              ///< 添加到输出点云
     }
  }
  dst = output;                                          ///< 设置最终输出结果
}

/**
 * @brief 使用中间点云标签对源点云进行标签映射
 * @param src 输入源点云（需要标签映射的点云）
 * @param medium 中间参考点云（提供标签信息的点云）
 * @param dst 输出点云（标签映射后的结果）
 * @param leaf_size 体素化网格大小
 * @details 该函数首先对源点云进行体素化下采样，然后通过KD树搜索在中间点云中找到最近邻点，
 *          将中间点云的标签信息映射到体素化后的源点云上
 */
void label_map(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr medium,
                          pcl::PointCloud<pcl::PointXYZI>& dst,
                          double leaf_size){
  pcl::PointCloud<pcl::PointXYZI> tmp, output;           ///< 临时点云和输出点云容器

  // ==================== 第一步：源点云体素化下采样 ====================
  static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;   ///< 静态体素网格滤波器
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>); ///< 体素化结果容器
  voxel_filter.setInputCloud(src);                      ///< 设置源点云作为输入
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size); ///< 设置体素大小
  voxel_filter.filter(*ptr_voxelized);                  ///< 执行体素化滤波
  tmp = *ptr_voxelized;                                  ///< 复制体素化结果

  // ==================== 第二步：从中间点云映射标签 ====================
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;              ///< KD树用于最近邻搜索
  kdtree.setInputCloud(medium);                         ///< 设置中间点云作为搜索树

  int K = 1;                                             ///< 搜索最近的1个点
  std::vector<int> pointIdxNKNSearch(K);                ///< 最近邻点索引容器
  std::vector<float> pointNKNSquaredDistance(K);        ///< 最近邻距离平方容器
  
  // 为每个体素化后的点从中间点云获取标签
  for (const auto &pt: tmp.points){                     ///< 遍历体素化后的每个点
    if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) ///< 在中间点云中搜索最近邻
    {
          auto updated = pt;                             ///< 复制当前点
          updated.intensity = (*medium)[pointIdxNKNSearch[0]].intensity; ///< 用中间点云最近邻点的标签更新
          output.points.push_back(updated);              ///< 添加到输出点云
     }
  }
  dst = output;                                          ///< 设置最终输出结果
}

/**
 * @brief 地图比较可视化主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 * @details 该程序加载多种动态对象移除算法的处理结果，解析动态/静态点云，
 *          并通过ROS话题发布用于可视化比较不同算法的效果
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapviz");                       ///< 初始化ROS节点
    std::cout<<"KiTTI MAPVIZ STARTED"<<std::endl;          ///< 输出启动信息
    ros::NodeHandle nodeHandler;                           ///< 创建ROS节点句柄

    // ==================== 参数配置 ====================
    std::string rawName, octoMapName, pplName, removertName, erasorName; ///< 各算法结果文件路径

    nodeHandler.param<std::string>("/raw", rawName, "/media/shapelim");           ///< 原始地图文件路径
    nodeHandler.param<std::string>("/octoMap", octoMapName, "/media/shapelim");   ///< OctoMap结果文件路径
    nodeHandler.param<std::string>("/pplremover", pplName, "/media/shapelim");   ///< PPL移除器结果文件路径
    nodeHandler.param<std::string>("/removert", removertName, "/media/shapelim"); ///< Removert结果文件路径
    nodeHandler.param<std::string>("/erasor", erasorName, "/media/shapelim");    ///< ERASOR结果文件路径


    // ==================== ROS发布器创建 ====================
    ros::Publisher msPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/static", 100);      ///< 原始地图静态点云发布器
    ros::Publisher mdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/dynamic", 100);     ///< 原始地图动态点云发布器

    ros::Publisher osPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/static", 100);  ///< OctoMap静态点云发布器
    ros::Publisher odPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/dynamic", 100); ///< OctoMap动态点云发布器

    ros::Publisher psPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/static", 100);  ///< PPL移除器静态点云发布器
    ros::Publisher pdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/dynamic", 100); ///< PPL移除器动态点云发布器

    ros::Publisher rsPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/static", 100);  ///< Removert静态点云发布器
    ros::Publisher rdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/dynamic", 100); ///< Removert动态点云发布器

    ros::Publisher esPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/static", 100);   ///< ERASOR静态点云发布器
    ros::Publisher edPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/dynamic", 100);  ///< ERASOR动态点云发布器


    // ==================== 点云数据加载 ====================
    std::cout<<"\033[1;32mLoading map..."<<std::endl;        ///< 输出加载开始信息
    
    // 加载原始地图点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);     ///< 原始地图点云容器
    datahandle3d::load_pcd(rawName, ptr_src);                                              ///< 加载原始地图PCD文件

    // 加载OctoMap处理结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_octo(new pcl::PointCloud<pcl::PointXYZI>);   ///< OctoMap结果点云容器
    datahandle3d::load_pcd(octoMapName, ptr_octo);                                         ///< 加载OctoMap结果PCD文件

    // 加载PPL移除器处理结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_ppl(new pcl::PointCloud<pcl::PointXYZI>);    ///< PPL移除器结果点云容器
    datahandle3d::load_pcd(pplName, ptr_ppl);                                              ///< 加载PPL移除器结果PCD文件

    // 加载Removert处理结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_removert(new pcl::PointCloud<pcl::PointXYZI>); ///< Removert结果点云容器
    datahandle3d::load_pcd(removertName, ptr_removert);                                    ///< 加载Removert结果PCD文件

    // 加载ERASOR处理结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_ours(new pcl::PointCloud<pcl::PointXYZI>);   ///< ERASOR结果点云容器
    datahandle3d::load_pcd(erasorName, ptr_ours);                                          ///< 加载ERASOR结果PCD文件
    std::cout<<"\033[1;32mLoad complete \033[0m"<<std::endl;  ///< 输出加载完成信息

    // ==================== 动态/静态点云解析 ====================
    pcl::PointCloud<pcl::PointXYZI> mapStatic, mapDynamic;           ///< 原始地图静态/动态点云
    pcl::PointCloud<pcl::PointXYZI> octoStatic, octoDynamic;         ///< OctoMap静态/动态点云
    pcl::PointCloud<pcl::PointXYZI> pplStatic, pplDynamic;           ///< PPL移除器静态/动态点云
    pcl::PointCloud<pcl::PointXYZI> removertStatic, removertDynamic; ///< Removert静态/动态点云
    pcl::PointCloud<pcl::PointXYZI> erasorStatic, erasorDynamic;     ///< ERASOR静态/动态点云

    // 解析各算法结果中的动态和静态对象
    parse_dynamic_obj(*ptr_ours, erasorDynamic, erasorStatic);       ///< 解析ERASOR结果
    parse_dynamic_obj(*ptr_removert, removertDynamic, removertStatic); ///< 解析Removert结果
    parse_dynamic_obj(*ptr_ppl, pplDynamic, pplStatic);              ///< 解析PPL移除器结果
    parse_dynamic_obj(*ptr_octo, octoDynamic, octoStatic);           ///< 解析OctoMap结果
    parse_dynamic_obj(*ptr_src, mapDynamic, mapStatic);              ///< 解析原始地图

    // ==================== 点云转ROS消息 ====================
    auto esmsg = erasor_utils::cloud2msg(erasorStatic);              ///< ERASOR静态点云消息
    auto edmsg = erasor_utils::cloud2msg(erasorDynamic);             ///< ERASOR动态点云消息
    auto rsmsg = erasor_utils::cloud2msg(removertStatic);            ///< Removert静态点云消息
    auto rdmsg = erasor_utils::cloud2msg(removertDynamic);           ///< Removert动态点云消息
    auto psmsg = erasor_utils::cloud2msg(pplStatic);                 ///< PPL移除器静态点云消息
    auto pdmsg = erasor_utils::cloud2msg(pplDynamic);                ///< PPL移除器动态点云消息
    auto osmsg = erasor_utils::cloud2msg(octoStatic);                ///< OctoMap静态点云消息
    auto odmsg = erasor_utils::cloud2msg(octoDynamic);               ///< OctoMap动态点云消息
    auto msmsg = erasor_utils::cloud2msg(mapStatic);                 ///< 原始地图静态点云消息
    auto mdmsg = erasor_utils::cloud2msg(mapDynamic);                ///< 原始地图动态点云消息

    // ==================== 主循环发布 ====================
    ros::Rate loop_rate(2);                                 ///< 设置发布频率为2Hz
    static int count_ = 0;                                   ///< 发布计数器
    while (ros::ok())                                        ///< ROS主循环
    {
      // 发布所有算法的静态和动态点云结果
      esPublisher.publish(esmsg);       edPublisher.publish(edmsg);   ///< 发布ERASOR结果
      rsPublisher.publish(rsmsg);       rdPublisher.publish(rdmsg);   ///< 发布Removert结果
      psPublisher.publish(psmsg);       pdPublisher.publish(pdmsg);   ///< 发布PPL移除器结果
      osPublisher.publish(osmsg);       odPublisher.publish(odmsg);   ///< 发布OctoMap结果
      msPublisher.publish(msmsg);       mdPublisher.publish(mdmsg);   ///< 发布原始地图结果

      if (++count_ %2 == 0) std::cout<<"On "<<count_<<"th publish!"<<std::endl; ///< 每2次发布输出一次状态

      ros::spinOnce();                                       ///< 处理ROS回调函数
      loop_rate.sleep();                                     ///< 按设定频率休眠
    }
    return 0;                                                ///< 程序正常退出
}
