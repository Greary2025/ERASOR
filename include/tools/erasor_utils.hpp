/**
 * @file erasor_utils.hpp
 * @brief ERASOR工具函数头文件
 * @author ERASOR团队
 * @description 定义ERASOR算法所需的各种工具函数，包括点云处理、坐标变换、
 *              ROS消息转换、动态对象解析等核心功能
 */

#ifndef ERASOR_UTILS_H
#define ERASOR_UTILS_H

// ==================== 标准C++库 ====================
#include <cstdlib>                                               ///< 标准库函数
#include <ctime>                                                 ///< 时间处理
#include <algorithm>                                             ///< 算法库
#include <sstream>                                               ///< 字符串流
#include <fstream>                                               ///< 文件流
#include <cmath>                                                 ///< 数学函数
#include <string>                                                ///< 字符串处理
#include <map>                                                   ///< 映射容器
#include <iostream>                                              ///< 输入输出流
#include <vector>                                                ///< 动态数组
#include <memory>                                                ///< 智能指针

// ==================== ROS相关库 ====================
#include <ros/ros.h>                                             ///< ROS核心功能

// ==================== PCL点云处理库 ====================
#include <pcl/common/common.h>                                   ///< PCL通用功能
#include <pcl/common/centroid.h>                                 ///< 质心计算
#include <pcl/common/transforms.h>                               ///< 点云变换
#include <pcl_conversions/pcl_conversions.h>                     ///< PCL与ROS消息转换
#include <pcl/point_types.h>                                     ///< 点云数据类型
#include <pcl/point_cloud.h>                                     ///< 点云基础类
#include <pcl/io/pcd_io.h>                                       ///< PCD文件输入输出
#include <pcl/filters/filter.h>                                  ///< 滤波器基类
#include <pcl/filters/voxel_grid.h>                              ///< 体素网格滤波器
#include <pcl/filters/passthrough.h>                             ///< 直通滤波器
#include <pcl/filters/extract_indices.h>                         ///< 索引提取滤波器
#include <pcl/kdtree/kdtree_flann.h>                             ///< KD树搜索
#include <pcl/ModelCoefficients.h>                               ///< 模型系数
#include <pcl/filters/approximate_voxel_grid.h>                  ///< 近似体素网格滤波器
#include <pcl/PCLPointCloud2.h>                                  ///< PCL点云消息类型
#include <pcl/visualization/pcl_visualizer.h>                    ///< 点云可视化

// ==================== ROS消息类型 ====================
#include <erasor/node.h>                                         ///< ERASOR节点消息
#include <visualization_msgs/Marker.h>                           ///< 可视化标记消息
#include <jsk_recognition_msgs/PolygonArray.h>                   ///< 多边形数组消息
#include <geometry_msgs/PolygonStamped.h>                        ///< 带时间戳的多边形消息
#include <geometry_msgs/Polygon.h>                               ///< 多边形消息
#include <geometry_msgs/Point.h>                                 ///< 点消息
#include <geometry_msgs/Point32.h>                               ///< 32位点消息
#include <nav_msgs/Path.h>                                       ///< 路径消息
#include <nav_msgs/Odometry.h>                                   ///< 里程计消息
#include <sensor_msgs/PointCloud2.h>                            ///< 点云消息
#include <tf/transform_broadcaster.h>                            ///< TF变换广播器
#include <std_msgs/Float32.h>                                    ///< 浮点数消息
#include <std_msgs/Int32.h>                                      ///< 整数消息
#include "signal.h"                                              ///< 信号处理


/**
 * @namespace erasor_utils
 * @brief ERASOR工具函数命名空间
 * @description 包含ERASOR算法所需的各种工具函数，提供点云处理、坐标变换、
 *              ROS消息转换、动态对象解析等核心功能
 */
namespace erasor_utils {
    
    /**
     * @brief 将PCL点云转换为ROS点云消息
     * @tparam T 点云数据类型（如pcl::PointXYZI）
     * @param cloud 输入的PCL点云
     * @param frame_id 坐标系ID，默认为"map"
     * @return sensor_msgs::PointCloud2 转换后的ROS点云消息
     * @description 将PCL格式的点云数据转换为ROS消息格式，便于在ROS系统中传输和处理
     */
    template<typename T>
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
    {
        sensor_msgs::PointCloud2 cloud_ROS;  ///< ROS点云消息容器
        pcl::toROSMsg(cloud, cloud_ROS);     ///< PCL到ROS消息转换
        cloud_ROS.header.frame_id = frame_id; ///< 设置坐标系ID
        return cloud_ROS;
    }

    /**
     * @brief 将ROS点云消息转换为PCL点云
     * @tparam T 点云数据类型（如pcl::PointXYZI）
     * @param cloudmsg 输入的ROS点云消息
     * @return pcl::PointCloud<T> 转换后的PCL点云
     * @description 将ROS消息格式的点云数据转换为PCL格式，便于使用PCL库进行处理
     */
    template<typename T>
    pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
    {
        pcl::PointCloud<T> cloudresult;       ///< PCL点云结果容器
        pcl::fromROSMsg(cloudmsg,cloudresult); ///< ROS消息到PCL转换
        return cloudresult;
    }

    /**
     * @brief 将ROS点云消息转换为PCL点云智能指针
     * @tparam T 点云数据类型（如pcl::PointXYZI）
     * @param cloudmsg 输入的ROS点云消息
     * @param cloudPtr 输出的PCL点云智能指针
     * @description 将ROS消息格式的点云数据转换为PCL智能指针格式，便于内存管理
     */
    template<typename T>
    void cloudmsg2cloudptr(sensor_msgs::PointCloud2 cloudmsg,boost::shared_ptr< pcl::PointCloud< T > > cloudPtr)
    {
        pcl::fromROSMsg(cloudmsg,*cloudPtr);  ///< ROS消息到PCL智能指针转换
    }


    /**
     * @brief 从PCD文件加载点云数据到智能指针
     * @tparam T 点云数据类型（如pcl::PointXYZI）
     * @param pcd_name PCD文件路径
     * @param dst 输出的点云智能指针
     * @return int 加载状态（0：成功，-1：失败）
     * @description 从指定路径的PCD文件中加载点云数据到智能指针容器
     */
    template<typename T>
    int load_pcd(std::string pcd_name, boost::shared_ptr<pcl::PointCloud<T> > dst) {
        if (pcl::io::loadPCDFile<T>(pcd_name, *dst) == -1) {  ///< 尝试加载PCD文件
            PCL_ERROR ("Couldn't read file!!! \n");           ///< 输出错误信息
            return (-1);                                       ///< 返回失败状态
        }
//        std::cout << "Loaded " << dst->size() << " data points from " << pcd_name << std::endl;
        return 0;                                              ///< 返回成功状态
    }

    /**
     * @brief 从PCD文件加载点云数据到引用对象
     * @tparam T 点云数据类型（如pcl::PointXYZI）
     * @param pcd_name PCD文件路径
     * @param dst 输出的点云引用对象
     * @return int 加载状态（0：成功，-1：失败）
     * @description 从指定路径的PCD文件中加载点云数据到引用对象，并输出加载信息
     */
    template<typename T>
    int load_pcd(std::string pcd_name, pcl::PointCloud<T>& dst) {
        if (pcl::io::loadPCDFile<T>(pcd_name, dst) == -1) {   ///< 尝试加载PCD文件
            PCL_ERROR ("Couldn't read file!!! \n");           ///< 输出错误信息
            return (-1);                                       ///< 返回失败状态
        }
        std::cout << "Loaded " << dst.size() << " data points from " << pcd_name << std::endl; ///< 输出加载信息
        return 0;                                              ///< 返回成功状态
    }

    /**
     * @brief 将Eigen变换矩阵转换为ROS几何位姿
     * @param pose 输入的Eigen 4x4变换矩阵
     * @return geometry_msgs::Pose 转换后的ROS几何位姿
     * @description 将Eigen格式的4x4变换矩阵转换为ROS几何位姿消息格式
     */
    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose);

    /**
     * @brief 将ROS几何位姿转换为Eigen变换矩阵
     * @param geoPose 输入的ROS几何位姿
     * @return Eigen::Matrix4f 转换后的Eigen 4x4变换矩阵
     * @description 将ROS几何位姿消息格式转换为Eigen格式的4x4变换矩阵
     */
    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose);

    /**
     * @brief 解析点云中的动态和静态对象
     * @param cloudIn 输入的点云数据
     * @param dynamicOut 输出的动态对象点云
     * @param staticOut 输出的静态对象点云
     * @description 根据点云中的语义标签信息，将点云分离为动态对象和静态对象两部分
     */
    void parse_dynamic_obj(
            const pcl::PointCloud<pcl::PointXYZI> &cloudIn, pcl::PointCloud<pcl::PointXYZI> &dynamicOut,
            pcl::PointCloud<pcl::PointXYZI> &staticOut);

    /**
     * @brief 体素化处理并保留语义标签
     * @param src 输入的源点云智能指针
     * @param dst 输出的体素化点云
     * @param leaf_size 体素网格叶子节点大小
     * @description 对点云进行体素化下采样处理，同时保留每个点的语义标签信息
     */
    void voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size);

    /**
     * @brief 统计点云中静态和动态点的数量
     * @param cloudIn 输入的点云数据
     * @param num_static 输出的静态点数量
     * @param num_dynamic 输出的动态点数量
     * @description 根据点云中的语义标签统计静态点和动态点的数量
     */
    void count_stat_dyn(const pcl::PointCloud<pcl::PointXYZI> &cloudIn, int &num_static, int &num_dynamic);

    /**
     * @brief 信号回调处理函数
     * @param signum 信号编号
     * @description 处理系统信号（如SIGINT），用于程序的优雅退出
     */
    void signal_callback_handler(int signum);

}
#endif // ERASOR_UTILS_H
