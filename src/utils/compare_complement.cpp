/**
 * @file compare_complement.cpp
 * @brief 地图补充比较工具
 * @author ERASOR团队
 * @description 该文件实现了多种动态对象移除算法的结果比较功能，
 *              包括ERASOR、Removert、PPLRemover、OctoMap等算法的静态地图补充分析
 */

// ==================== ROS相关库 ====================
#include <ros/ros.h>                                    ///< ROS核心功能

// ==================== PCL点云处理库 ====================
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

// ==================== 自定义工具库 ====================
#include <unavlib/convt.h>                              ///< 数据转换工具
#include <unavlib/others.h>                             ///< 其他实用工具

// ==================== 标准C++库 ====================
#include <string>                                       ///< 字符串处理
#include <map>                                          ///< 映射容器
#include <vector>                                       ///< 动态数组

using namespace unavlib;

// ==================== 全局变量定义 ====================
/**
 * @brief 动态对象类别标签定义
 * @description 基于SemanticKITTI数据集的动态对象语义标签
 *              包含车辆、行人、自行车等移动对象的标签ID
 */
std::vector<int> dynamic_classes = {252, 253, 254, 255, 256, 257, 258, 259};

/**
 * @brief 解析动态对象点云
 * @param cloudIn 输入的带语义标签的点云
 * @param dynamicOut 输出的动态对象点云
 * @param staticOut 输出的静态对象点云
 * @description 根据SemanticKITTI语义标签将输入点云分离为动态和静态两部分，
 *              通过解析点的强度值中编码的语义信息进行分类
 */
void parse_dynamic_obj(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, pcl::PointCloud<pcl::PointXYZI>& dynamicOut, pcl::PointCloud<pcl::PointXYZI>& staticOut){
  // ==================== 初始化输出点云 ====================
  dynamicOut.points.clear();                             ///< 清空动态对象点云
  staticOut.points.clear();                              ///< 清空静态对象点云

  // ==================== 遍历处理每个点 ====================
  for (const auto &pt: cloudIn.points){
    // ==================== 解析语义标签信息 ====================
    uint32_t float2int = static_cast<uint32_t>(pt.intensity);  ///< 将强度值转换为整数
    uint32_t semantic_label = float2int & 0xFFFF;              ///< 提取语义标签（低16位）
    uint32_t inst_label = float2int >> 16;                     ///< 提取实例标签（高16位）
    
    // ==================== 动态对象分类检测 ====================
    bool is_static = true;                                     ///< 默认为静态对象
    for (int class_num: dynamic_classes){
      if (semantic_label == class_num){                        ///< 检查是否属于动态对象类别
        dynamicOut.points.push_back(pt);                      ///< 添加到动态对象点云
        is_static = false;                                     ///< 标记为动态对象
      }
    }
    
    // ==================== 静态对象归类 ====================
    if (is_static){
      staticOut.points.push_back(pt);                          ///< 添加到静态对象点云
    }
  }
}
/**
 * @brief 计算地图补充点云
 * @param src 估计的静态地图点云（算法处理结果）
 * @param gt 真实地图点云（原始数据）
 * @param complement 输出的补充点云（缺失的静态点）
 * @description 通过比较估计地图和真实地图，找出被错误移除的静态点，
 *              使用KD树进行最近邻搜索，并根据距离阈值判断是否为缺失点
 */
void calc_complement(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr gt,
                          pcl::PointCloud<pcl::PointXYZI>& complement){
  // ==================== 初始化补充点云 ====================
  if (!complement.empty()) complement.clear();            ///< 清空补充点云
  
  // ==================== 构建KD树索引 ====================
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;               ///< 创建KD树用于快速最近邻搜索
  kdtree.setInputCloud(src);                              ///< 设置估计地图为搜索目标

  // ==================== 最近邻搜索参数 ====================
  int K = 1;                                              ///< 搜索最近的1个邻居
  std::vector<int> pointIdxNKNSearch(K);                  ///< 存储最近邻点索引
  std::vector<float> pointNKNSquaredDistance(K);          ///< 存储最近邻距离的平方

  // ==================== 遍历真实地图中的每个点 ====================
  for (const auto &pt: gt->points){
    // ==================== 执行最近邻搜索 ====================
    if ( kdtree.nearestKSearch (pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        // ==================== 解析语义标签信息 ====================
        uint32_t float2int = static_cast<uint32_t>(pt.intensity);  ///< 将强度值转换为整数
        uint32_t semantic_label = float2int & 0xFFFF;              ///< 提取语义标签（低16位）
        uint32_t inst_label = float2int >> 16;                     ///< 提取实例标签（高16位）
        
        // ==================== 静态对象检测 ====================
        bool is_static = true;                                     ///< 默认为静态对象
        for (int class_num: dynamic_classes){
          if (semantic_label == class_num){                        ///< 检查是否属于动态对象类别
            is_static = false;                                     ///< 标记为动态对象
          }
        }
        
        // ==================== 补充点判断 ====================
        // 如果是静态点且距离最近邻超过阈值（3cm），则认为是被错误移除的点
        if (is_static && (pointNKNSquaredDistance[0] > 0.03)){
          complement.points.push_back(pt);                         ///< 添加到补充点云
        }
     }
  }
}

/**
 * @brief 主程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态码
 * @description 多算法地图补充比较工具的主程序，用于比较不同动态对象移除算法的性能
 */
int main(int argc, char **argv)
{
    // ==================== ROS节点初始化 ====================
    ros::init(argc, argv, "mapviz");                       ///< 初始化ROS节点
    std::cout<<"KiTTI MAPVIZ STARTED"<<std::endl;          ///< 输出启动信息
    ros::NodeHandle nodeHandler;                            ///< 创建ROS节点句柄

    // ==================== 参数读取 ====================
    std::string rawName, octoMapName, pplName, removertName, erasorName;  ///< 各算法结果文件路径

    nodeHandler.param<std::string>("/raw", rawName, "/media/shapelim");           ///< 原始地图文件路径
    nodeHandler.param<std::string>("/octoMap", octoMapName, "/media/shapelim");   ///< OctoMap算法结果路径
    nodeHandler.param<std::string>("/pplremover", pplName, "/media/shapelim");   ///< PPLRemover算法结果路径
    nodeHandler.param<std::string>("/removert", removertName, "/media/shapelim"); ///< Removert算法结果路径
    nodeHandler.param<std::string>("/erasor", erasorName, "/media/shapelim");     ///< ERASOR算法结果路径


    // ==================== ROS发布器设置 ====================
    ros::Publisher msPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/static", 100);        ///< 原始地图静态点云发布器
    ros::Publisher mdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/dynamic", 100);       ///< 原始地图动态点云发布器

    ros::Publisher osPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/static", 100);    ///< OctoMap静态点云发布器
    ros::Publisher odPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/dynamic", 100);   ///< OctoMap动态点云发布器

    ros::Publisher psPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/static", 100); ///< PPLRemover静态点云发布器
    ros::Publisher pdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/dynamic", 100);///< PPLRemover动态点云发布器

    ros::Publisher rsPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/static", 100);   ///< Removert静态点云发布器
    ros::Publisher rdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/dynamic", 100);  ///< Removert动态点云发布器

    ros::Publisher esPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/static", 100);     ///< ERASOR静态点云发布器
    ros::Publisher edPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/dynamic", 100);    ///< ERASOR动态点云发布器


    // ==================== 地图数据加载 ====================
    std::cout<<"\033[1;32mLoading map..."<<std::endl;        ///< 输出加载开始信息
    
    // 加载原始真实地图数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);     ///< 原始地图点云指针
    datahandle3d::load_pcd(rawName, ptr_src);                                              ///< 加载原始地图PCD文件

    // 加载OctoMap算法处理结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_octo(new pcl::PointCloud<pcl::PointXYZI>);   ///< OctoMap结果点云指针
    datahandle3d::load_pcd(octoMapName, ptr_octo);                                         ///< 加载OctoMap结果PCD文件

    // 加载PPLRemover算法处理结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_ppl(new pcl::PointCloud<pcl::PointXYZI>);    ///< PPLRemover结果点云指针
    datahandle3d::load_pcd(pplName, ptr_ppl);                                              ///< 加载PPLRemover结果PCD文件

    // 加载Removert算法处理结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_removert(new pcl::PointCloud<pcl::PointXYZI>); ///< Removert结果点云指针
    datahandle3d::load_pcd(removertName, ptr_removert);                                    ///< 加载Removert结果PCD文件

    // 加载ERASOR算法处理结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_ours(new pcl::PointCloud<pcl::PointXYZI>);   ///< ERASOR结果点云指针
    datahandle3d::load_pcd(erasorName, ptr_ours);                                          ///< 加载ERASOR结果PCD文件
    std::cout<<"\033[1;32mLoad complete \033[0m"<<std::endl;  ///< 输出加载完成信息

    // ==================== 补充点云计算 ====================
    pcl::PointCloud<pcl::PointXYZI> mapStatic, mapDynamic;         ///< 原始地图静态和动态点云
    pcl::PointCloud<pcl::PointXYZI> octoStatic, octoDynamic;       ///< OctoMap补充静态和动态点云
    pcl::PointCloud<pcl::PointXYZI> pplStatic, pplDynamic;         ///< PPLRemover补充静态和动态点云
    pcl::PointCloud<pcl::PointXYZI> removertStatic, removertDynamic; ///< Removert补充静态和动态点云
    pcl::PointCloud<pcl::PointXYZI> erasorStatic, erasorDynamic;   ///< ERASOR补充静态和动态点云

    // 计算各算法相对于原始地图的补充点云（被错误移除的静态点）
    calc_complement(ptr_octo, ptr_src, octoStatic);                 ///< 计算OctoMap的补充点云
    calc_complement(ptr_ppl, ptr_src, pplStatic);                   ///< 计算PPLRemover的补充点云
    calc_complement(ptr_removert, ptr_src, removertStatic);         ///< 计算Removert的补充点云
    calc_complement(ptr_ours, ptr_src, erasorStatic);               ///< 计算ERASOR的补充点云

    // ==================== 点云消息转换 ====================
    auto esmsg = erasor_utilscloud2msg(erasorStatic);               ///< 转换ERASOR补充点云为ROS消息
//    auto edmsg = erasor_utilscloud2msg(erasorDynamic);             ///< ERASOR动态点云消息（未使用）
    auto rsmsg = erasor_utilscloud2msg(removertStatic);             ///< 转换Removert补充点云为ROS消息
//    auto rdmsg = erasor_utilscloud2msg(removertDynamic);           ///< Removert动态点云消息（未使用）
    auto psmsg = erasor_utilscloud2msg(pplStatic);                  ///< 转换PPLRemover补充点云为ROS消息
//    auto pdmsg = erasor_utilscloud2msg(pplDynamic);                ///< PPLRemover动态点云消息（未使用）
    auto osmsg = erasor_utilscloud2msg(octoStatic);                 ///< 转换OctoMap补充点云为ROS消息
//    auto odmsg = erasor_utilscloud2msg(octoDynamic);               ///< OctoMap动态点云消息（未使用）
//    auto msmsg = erasor_utilscloud2msg(mapStatic);                 ///< 原始地图静态点云消息（未使用）
//    auto mdmsg = erasor_utilscloud2msg(mapDynamic);                ///< 原始地图动态点云消息（未使用）

    // ==================== 循环发布点云数据 ====================
    ros::Rate loop_rate(2);                                         ///< 设置发布频率为2Hz
    static int count_ = 0;                                           ///< 发布计数器
    while (ros::ok())                                                ///< ROS主循环
    {
      esPublisher.publish(esmsg);                                    ///< 发布ERASOR补充点云
      rsPublisher.publish(rsmsg);                                    ///< 发布Removert补充点云
      psPublisher.publish(psmsg);                                    ///< 发布PPLRemover补充点云
      osPublisher.publish(osmsg);                                    ///< 发布OctoMap补充点云

      // ==================== 发布状态输出 ====================
      if (++count_ %2 == 0) std::cout<<"On "<<count_<<"th publish!"<<std::endl; ///< 每2次发布输出一次状态

      ros::spinOnce();                                               ///< 处理ROS回调
      loop_rate.sleep();                                             ///< 按设定频率休眠
    }
    return 0;                                                        ///< 程序正常退出
}
