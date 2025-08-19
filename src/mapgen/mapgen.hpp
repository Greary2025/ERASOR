/**
 * @file mapgen.hpp
 * @brief KITTI数据集地图生成器头文件
 * @author Hyungtae Lim (shapelim@kaist.ac.kr)
 * @description 用于从KITTI数据集生成大规模点云地图的工具类
 *              支持动态对象检测和地图构建功能
 */

#ifndef MAPGEN_H
#define MAPGEN_H

// ==================== 标准库头文件 ====================
#include <iostream>                                              ///< 输入输出流
#include <string>                                                ///< 字符串处理
#include <vector>                                                ///< 动态数组容器
#include <ctime>                                                 ///< 时间处理
#include <cassert>                                               ///< 断言宏
#include <cmath>                                                 ///< 数学函数
#include <utility>                                               ///< 实用工具
#include <algorithm>                                             ///< 算法库
#include <cstdlib>                                               ///< 标准库函数
#include <memory>                                                ///< 智能指针
#include <iomanip>                                               ///< 输入输出格式控制

// ==================== ROS相关头文件 ====================
#include <ros/ros.h>                                             ///< ROS核心功能
#include <sensor_msgs/PointCloud2.h>                            ///< 点云消息类型
#include <geometry_msgs/PoseStamped.h>                          ///< 带时间戳的位姿消息
#include <nav_msgs/Path.h>                                       ///< 路径消息类型
#include <std_msgs/Float32.h>                                    ///< 浮点数消息类型

// ==================== PCL相关头文件 ====================
#include <pcl_conversions/pcl_conversions.h>                    ///< PCL与ROS消息转换
#include <pcl/point_cloud.h>                                    ///< PCL点云类
#include <pcl/point_types.h>                                    ///< PCL点类型定义
#include <pcl/filters/voxel_grid.h>                             ///< 体素网格滤波器
#include <pcl/io/pcd_io.h>                                      ///< PCD文件输入输出

// ==================== 自定义工具头文件 ====================
#include <erasor/node.h>                                        ///< ERASOR节点定义
#include "tools/erasor_utils.hpp"                               ///< ERASOR工具函数
#include <math.h>                                               ///< 数学函数库

// ==================== 常量定义 ====================
#define CAR_BODY_SIZE 2.7                                       ///< 车体尺寸常量(米)
#define NUM_MAP_PC_LARGE_ENOUGH 30000000                        ///< 大规模地图点云数量上限

/**
 * @struct Cluster
 * @brief 点云聚类结构体
 * @description 用于存储动态对象检测中的聚类信息
 */
struct Cluster {
    double   x;                                                 ///< 聚类质心X坐标
    double   y;                                                 ///< 聚类质心Y坐标
    double   z;                                                 ///< 聚类质心Z坐标
    int      num_pt = 0;                                       ///< 聚类中点的数量
    uint32_t class_num;                                         ///< 语义类别标签
};

// ==================== 全局常量 ====================
const char separator    = ' ';                              ///< 输出格式分隔符
const int nameWidth     = 10;                               ///< 输出格式名称宽度

/**
 * @brief 计算两个聚类间的欧几里得距离
 * @param d0 第一个聚类
 * @param d1 第二个聚类
 * @return double 两聚类质心间的距离
 * @description 用于动态对象运动检测的距离计算函数
 * 
 * 功能说明：
 * 计算三维空间中两个聚类质心之间的直线距离，用于动态对象运动检测
 */
double calc_dist(Cluster d0, Cluster d1) {
    return sqrt(pow(d0.x - d1.x, 2) + pow(d0.y - d1.y, 2) + pow(d0.z - d1.z, 2));  ///< 三维欧几里得距离公式
}

/**
 * @class mapgen
 * @brief KITTI数据集地图生成器类
 * @description 用于从KITTI数据集生成大规模点云地图，支持动态对象检测和地图构建功能
 */
class mapgen {
private:
    // ==================== 基础状态变量 ====================
    bool                            is_initial  = true;        ///< 是否为初始状态标志
    int                             count       = 0;           ///< 当前帧计数器
    int                             accum_count = 0;           ///< 累积帧计数器
    int                             interval, last_ts, init_ts; ///< 帧间隔、最后时间戳、初始时间戳
    
    // ==================== 点云数据 ====================
    pcl::PointCloud<pcl::PointXYZI> cloud_curr;               ///< 当前帧点云
    pcl::PointCloud<pcl::PointXYZI> cloud_map;                ///< 累积地图点云

    // ==================== 大规模地图构建 ====================
    // ROS发布的点云数据量限制在1GB以下
    std::vector<pcl::PointCloud<pcl::PointXYZI> > cloud_maps;  ///< 多个子地图点云容器

    // ==================== 路径和参数 ====================
    nav_msgs::Path odom_path;                                  ///< 里程计路径
    float          leafsize;                                   ///< 体素网格叶子节点大小
    std::string    seq, init_stamp, final_stamp, save_path;    ///< 序列号、初始时间戳、结束时间戳、保存路径

    bool is_large_scale;                                       ///< 是否为大规模地图标志

    // ==================== 动态对象检测相关 ====================
    double                               movement_thr    = 1.0; ///< 运动阈值(米)
    std::map<int, std::vector<Cluster> > not_moving_object_candidates; ///< 非运动对象候选 <id, Cluster>
    std::vector<int>                     dynamic_classes = {252, 253, 254, 255, 256, 257, 258, 259}; ///< 动态类别标签列表
    std::map<int, int>                   dynamic_objects; ///< 动态对象映射 <id, class_num> 表示运动对象的ID
    std::vector<int>                     all_dynamic_ids; ///< 所有动态对象ID列表
    std::vector<int>                     moving_dynamic_ids; ///< 运动中的动态对象ID列表

    /**
     * @brief 检测动态对象的运动状态
     * @param cloud 输入点云，包含语义和实例标签信息
     * 
     * 功能说明：
     * 1. 从点云中提取动态类别的对象实例
     * 2. 计算每个实例的质心位置
     * 3. 跟踪对象在多帧间的运动轨迹
     * 4. 识别真正运动的动态对象
     * 
     * 总结：静态对象 = {not_moving_object_candidates} - {dynamic_objects}
     */
    void check_movement(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
        std::map<int, Cluster> clusters_tmp; // <实例ID, 聚类信息> 临时存储当前帧的聚类结果
        
        // 第一步：将点云转换为 <实例ID, 质心> 的映射关系
        for (const auto        &pt: cloud.points) {
            // 从点的强度值中解析语义标签和实例标签
            uint32_t float2int      = static_cast<uint32_t>(pt.intensity);  // 将浮点强度转为整数
            uint32_t semantic_label = float2int & 0xFFFF;                   // 低16位为语义标签
            uint32_t inst_label     = float2int >> 16;                      // 高16位为实例标签

            // 检查是否属于动态类别
            for (int class_num: dynamic_classes) {
                if (semantic_label == class_num) { // 1. 检查是否属于动态对象类别
                    if (!clusters_tmp.empty()) {
                        // 查找是否已存在该实例ID的聚类
                        for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
                            if (it->first == inst_label) { // 实例已存在，累加坐标
                                it->second.x      = it->second.x + pt.x;      // 累加X坐标
                                it->second.y      = it->second.y + pt.y;      // 累加Y坐标
                                it->second.z      = it->second.z + pt.z;      // 累加Z坐标
                                it->second.num_pt = it->second.num_pt + 1;    // 增加点数计数
                            } else {
                                // 创建新的聚类实例
                                Cluster instance = {pt.x, pt.y, pt.z, 1, semantic_label};
                                clusters_tmp[inst_label] = instance;
                            }
                        }
                    } else { // 初始化：第一个聚类
                        Cluster instance = {pt.x, pt.y, pt.z, 1, semantic_label};
                        clusters_tmp[inst_label] = instance;
                    }
                }
            }
        }
        // 第二步：计算每个聚类的质心坐标
        for (auto              it = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
            // 通过平均值计算质心坐标
            it->second.x = it->second.x / static_cast<double>(it->second.num_pt);  // 计算X坐标质心
            it->second.y = it->second.y / static_cast<double>(it->second.num_pt);  // 计算Y坐标质心
            it->second.z = it->second.z / static_cast<double>(it->second.num_pt);  // 计算Z坐标质心

            // 更新全局动态对象ID列表（去重处理）
            all_dynamic_ids.push_back(it->first);
            for (auto it_id = all_dynamic_ids.begin(); it_id != (all_dynamic_ids.end() - 1); it_id++) {
                if (it->first == *it_id) {  // 发现重复ID，移除最后添加的
                    all_dynamic_ids.pop_back();
                    break;
                }
            }
        }
        
        // 调试信息输出：显示当前帧检测到的动态对象ID
        std::cout << "\033[1;32m=========[Dynamic Ids]===========" << std::endl;
        for (auto it = all_dynamic_ids.begin(); it != all_dynamic_ids.end(); it++) {
            std::cout << *it << ", ";
        }
        std::cout << "are dynamic objects!" << std::endl;
        std::cout << "=================================\033[0m" << std::endl;
        std::cout << "[Cluster size]: " << clusters_tmp.size() << std::endl;

        // 输出每个聚类的质心坐标
        for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
            std::cout << it->first << ", " << it->second.x << ", " << it->second.y << ", " << it->second.z << std::endl;
        }

        // 第三步：更新非运动对象候选列表
        if (not_moving_object_candidates.empty()) { // 初始化：第一次运行
            // 将所有检测到的动态类别对象加入候选列表
            for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
                std::vector<Cluster> obj_trajectories = {it->second};  // 创建轨迹向量
                not_moving_object_candidates.insert(std::make_pair(it->first, obj_trajectories));
            }
            return;  // 第一帧无法判断运动，直接返回
        }

        // 调试输出：显示候选对象及其轨迹长度
        for (auto it_not_mv = not_moving_object_candidates.begin(); it_not_mv != not_moving_object_candidates.end(); it_not_mv++) {
            std::cout << it_not_mv->first << ", " << it_not_mv->second.size() << std::endl;
        }

        // 一般情况：更新现有对象轨迹或添加新对象
        for (auto        it        = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
            bool      is_in     = false;  // 标记是否为已存在的对象
            // 查找当前对象是否已在候选列表中
            for (auto it_not_mv = not_moving_object_candidates.begin(); it_not_mv != not_moving_object_candidates.end(); it_not_mv++) {
                if (it->first == it_not_mv->first) { // 找到匹配的对象ID
                    it_not_mv->second.push_back(it->second);  // 将当前位置添加到轨迹中
                    is_in = true;
                }
            }
            if (!is_in) { // 新出现的对象
                std::vector<Cluster> obj_trajectories = {it->second};  // 创建新的轨迹
                not_moving_object_candidates.insert(std::make_pair(it->first, obj_trajectories));
            }
        }
        // 第四步：检测对象运动状态
        for (auto        it_not_mv = not_moving_object_candidates.begin(); it_not_mv != not_moving_object_candidates.end(); it_not_mv++) {
            if (it_not_mv->second.size() > 1) {  // 至少需要两个位置点才能判断运动
                // 获取轨迹的第一个和最后一个位置
                Cluster first_obj = it_not_mv->second.front();  // 轨迹起始位置
                Cluster last_obj  = it_not_mv->second.back();   // 轨迹结束位置
                double  movement  = calc_dist(first_obj, last_obj);  // 计算总位移距离
                
                if (movement_thr < movement) { // 位移超过阈值，判定为动态对象
                    // 将对象标记为动态并记录其类别
                    dynamic_objects.insert(std::make_pair(it_not_mv->first, it_not_mv->second.front().class_num));
                    moving_dynamic_ids.push_back(it_not_mv->first);  // 添加到运动对象ID列表
                }
            }
        }
        
        // 识别静态对象：在动态类别中但未发生显著运动的对象
        std::vector<int> static_objects;
        for (auto        it_all    = all_dynamic_ids.begin(); it_all != all_dynamic_ids.end(); it_all++) {
            bool      moved = false;  // 标记是否发生运动
            // 检查该对象是否在运动对象列表中
            for (auto it    = moving_dynamic_ids.begin(); it != moving_dynamic_ids.end(); it++) {
                if (*it_all == *it) {
                    moved = true;  // 找到匹配，说明对象发生了运动
                }
            }
            if (!moved) {  // 未发生运动，归类为静态对象
                static_objects.push_back(*it_all);
            }
        }
        
        // 调试输出：显示识别出的静态对象
        std::cout << "\033[2;32m========================" << std::endl;
        if (!static_objects.empty()) {
            for (auto id:static_objects) {
                std::cout << id << ", ";
            }
            std::cout << "are static!" << std::endl;
        }
        std::cout << "========================\033[0m" << std::endl;
    }

public:
    /**
     * @brief mapgen类构造函数
     * 
     * 功能说明：
     * 1. 初始化地图点云容器
     * 2. 为大规模地图预分配内存空间
     */
    mapgen() {
        cloud_map.reserve(NUM_MAP_PC_LARGE_ENOUGH);  // 预分配足够大的内存空间
    }
    
    /**
     * @brief mapgen类析构函数
     */
    ~mapgen() {}

    /**
     * @brief 设置地图生成参数
     * @param pcd_save_path PCD文件保存路径
     * @param voxelsize 体素化网格大小
     * @param sequence 数据序列号
     * @param init_time_stamp 起始时间戳
     * @param final_time_stamp 结束时间戳
     * @param frame_interval 帧间隔
     * @param is_map_large_scale 是否为大规模地图模式
     * 
     * 功能说明：
     * 1. 配置地图生成的各项参数
     * 2. 设置时间范围和处理间隔
     * 3. 输出配置信息用于调试
     */
    void setValue(std::string pcd_save_path,float voxelsize, std::string sequence, std::string init_time_stamp, std::string final_time_stamp, int frame_interval, bool is_map_large_scale) {
        // 基本参数设置
        save_path   = pcd_save_path;      // 保存路径
        leafsize    = voxelsize;          // 体素网格大小
        seq         = sequence;           // 序列号
        init_stamp  = init_time_stamp;    // 起始时间戳
        final_stamp = final_time_stamp;   // 结束时间戳
        interval    = frame_interval;     // 帧处理间隔

        // 时间戳转换为整数
        last_ts     = std::stoi(final_stamp);  // 结束时间戳（整数）
        init_ts     = std::stoi(init_stamp);   // 起始时间戳（整数）

        // 大规模地图标志
        is_large_scale = is_map_large_scale;

        // 输出配置信息
        std::cout << "\033[1;32m";
        std::cout << "[MAPGEN]: Voxelization size - " << leafsize << std::endl;
        std::cout << "[MAPGEN]: Target seq -  " << seq << std::endl;
        std::cout << "[MAPGEN]: From " << init_stamp << ", " << final_stamp << std::endl;
        std::cout << "[MAPGEN]: Is the map large-scale? " << is_large_scale << "\033[0m" << std::endl;
    }

    /**
     * @brief 累积点云数据并构建地图
     * @param data ERASOR节点数据，包含激光雷达点云和里程计信息
     * @param path 输出的里程计路径
     * 
     * 功能说明：
     * 1. 处理里程计数据并构建路径
     * 2. 坐标变换：激光雷达坐标系 -> 车体坐标系 -> 世界坐标系
     * 3. 过滤车体附近的噪声点
     * 4. 累积点云构建地图
     * 5. 大规模地图模式下的分块处理
     */
    void accumPointCloud(
            const erasor::node &data, nav_msgs::Path &path) {
        // 构建带时间戳的位姿消息
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header          = data.header;     // 复制头信息
        pose_stamped.header.frame_id = "map";           // 设置坐标系为地图坐标系
        pose_stamped.pose            = data.odom;       // 复制里程计位姿

        // 更新里程计路径
        odom_path.header = pose_stamped.header;
        odom_path.poses.push_back(pose_stamped);        // 添加当前位姿到路径
        path = odom_path;                               // 输出路径
        
        // 定义激光雷达到车体原点的变换矩阵
        Eigen::Matrix4f tf_lidar2origin;
        tf_lidar2origin << 1, 0, 0, 0,      // X轴不变
                0, 1, 0, 0,                 // Y轴不变
                0, 0, 1, 1.73,              // Z轴偏移1.73米（激光雷达高度）
                0, 0, 0, 1;                 // 齐次坐标

        // 将ROS点云消息转换为PCL点云
        pcl::PointCloud<pcl::PointXYZI> cloud = erasor_utils::cloudmsg2cloud<pcl::PointXYZI>(data.lidar);

        // 移除车体附近的噪声点（避免车体自身反射）
        pcl::PointCloud<pcl::PointXYZI> inliers, outliers; // 相对于车体原点
        float                           max_dist_square = pow(CAR_BODY_SIZE, 2);  // 车体尺寸的平方
        for (auto const                 &pt : cloud.points) {
            double dist_square = pow(pt.x, 2) + pow(pt.y, 2);  // 计算到原点的距离平方
            if (dist_square < max_dist_square) {
                inliers.push_back(pt);   // 车体附近的点（噪声）
            } else {
                outliers.push_back(pt);  // 有效的环境点
            }
        }
        cloud = outliers;  // 使用过滤后的点云

        // 第一步坐标变换：激光雷达坐标系 -> 车体坐标系
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(cloud, *ptr_transformed, tf_lidar2origin);

        // 获取当前帧的世界坐标位姿
        Eigen::Matrix4f pose = erasor_utils::geoPose2eigen(data.odom);
        std::cout << std::setprecision(3) << std::left << setw(nameWidth) << setfill(separator) 
                  << "=> [Pose] " << pose(0, 3) << ", " << pose(1, 3) << ", " << pose(2, 3) << std::endl;
        
        // 第二步坐标变换：车体坐标系 -> 世界坐标系
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*ptr_transformed, *world_transformed, pose);

        // 体素化处理，保留语义标签（0.2米网格）
        erasor_utils::voxelize_preserving_labels(world_transformed, cloud_curr, 0.2);

        // 地图累积处理
        if (is_initial) {
            // 初始化：第一帧作为地图基础
            cloud_map  = cloud_curr;
            is_initial = false;
        } else {
            // 累积：将当前帧添加到地图中
            cloud_map += cloud_curr;

            // 大规模地图模式：分块处理避免内存溢出
            if (is_large_scale) {
                static int cnt_voxel = 0;  // 静态计数器
                if (cnt_voxel++ % 500 == 0){  // 每500帧进行一次分块处理
                    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
                    *ptr_map = cloud_map;
                    std::cout << "\033[1;32m Voxelizing submap...\033[0m" << std::endl;
                    // 对子地图进行体素化处理
                    erasor_utils::voxelize_preserving_labels(ptr_map, cloud_map, leafsize);

                    // 保存子地图并清空当前地图
                    cloud_maps.push_back(cloud_map);
                    cloud_map.clear();
                }
            }
            ++accum_count;  // 增加累积帧计数
        }
    }
    /**
     * @brief 获取当前地图点云和当前帧点云
     * @param map_out 输出的累积地图点云指针
     * @param curr_out 输出的当前帧点云指针
     * 
     * 功能说明：
     * 1. 将内部存储的累积地图点云复制到输出参数
     * 2. 将内部存储的当前帧点云复制到输出参数
     * 3. 用于外部模块获取点云数据进行进一步处理
     */
    void getPointClouds(pcl::PointCloud<pcl::PointXYZI>::Ptr map_out,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr curr_out){
        *map_out  = cloud_map;   ///< 复制累积地图点云
        *curr_out = cloud_curr;  ///< 复制当前帧点云
    }

    /**
     * @brief 保存原始地图和体素化地图到PCD文件
     * @param original_dir 原始地图保存路径
     * @param map_dir 体素化地图保存路径
     * 
     * 功能说明：
     * 1. 合并所有子地图生成完整的原始地图
     * 2. 保存未经体素化的原始地图到指定路径
     * 3. 对合并后的地图进行体素化处理
     * 4. 保存体素化后的地图到指定路径
     * 5. 支持大规模地图模式的分块合并处理
     * 
     * 处理流程：
     * - 大规模模式：合并所有子地图和当前地图
     * - 普通模式：直接使用当前累积地图
     * - 保存原始地图 -> 体素化处理 -> 保存体素化地图
     */
    void saveNaiveMap(const std::string& original_dir, const std::string& map_dir){
        pcl::PointCloud<pcl::PointXYZI> cloud_src;  ///< 源点云数据容器

        std::cout << "\033[1;32m On saving map cloud...it may take few seconds...\033[0m" << std::endl;
        if (is_large_scale){  ///< 大规模地图模式：合并所有子地图
            // 合并之前保存的子地图
            for (const auto & submap: cloud_maps){
                cloud_src += submap;  ///< 累加子地图点云
            }
            // 合并当前剩余的地图数据
            cloud_src += cloud_map;  ///< 累加当前地图点云
        }else{  ///< 普通模式：直接使用当前地图
            cloud_src = cloud_map;  ///< 复制当前地图点云
        }

        // 保存原始未处理的地图
        pcl::io::savePCDFileASCII(original_dir, cloud_src);  ///< 保存原始地图到PCD文件

        // 准备体素化处理
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);  ///< 智能指针容器
        pcl::PointCloud<pcl::PointXYZI> cloud_out;  ///< 体素化输出容器
        ptr_map->points.reserve(cloud_src.points.size());  ///< 预分配内存空间

        std::cout << "\033[1;32m Start to copy pts...\033[0m" << std::endl;
        *ptr_map = cloud_src;  ///< 复制点云数据到智能指针
        std::cout << "[Debug]: " << cloud_src.width << ", " << cloud_src.height << ", " << cloud_src.points.size() << std::endl;
        std::cout << "\033[1;32m On voxelizing...\033[0m" << std::endl;
        // 执行体素化处理，保留语义标签
        erasor_utils::voxelize_preserving_labels(ptr_map, cloud_out, leafsize);  ///< 体素化处理

        // 设置输出点云的尺寸信息
        cloud_out.width  = cloud_out.points.size();  ///< 设置点云宽度
        cloud_out.height = 1;  ///< 设置点云高度为1（无序点云）
        std::cout << "[Debug]: " << cloud_out.width << ", " << cloud_out.height << ", " << cloud_out.points.size() << std::endl;
        std::cout << "\033[1;32m Saving the map to pcd...\033[0m" << std::endl;
        // 保存体素化后的地图
        pcl::io::savePCDFileASCII(map_dir, cloud_out);  ///< 保存体素化地图到PCD文件
        std::cout << "\033[1;32m Complete to save the map!:";
        std::cout << map_dir << "\033[0m" << std::endl;

    }
};

#endif

