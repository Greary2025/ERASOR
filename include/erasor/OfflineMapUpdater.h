/**
 * @file OfflineMapUpdater.h
 * @brief ERASOR离线地图更新器头文件
 * @author ERASOR团队
 * @description 定义了OfflineMapUpdater类，用于离线处理和更新激光雷达地图，
 *              通过ERASOR算法去除动态对象，保留静态环境结构
 */

#ifndef OFFLINEMAPUPDATER_H
#define OFFLINEMAPUPDATER_H

// ==================== ROS相关头文件 ====================
#include <ros/ros.h>                    ///< ROS核心功能
#include <sensor_msgs/PointCloud2.h>    ///< 点云消息类型
#include <geometry_msgs/PoseStamped.h>  ///< 位姿消息类型
#include <nav_msgs/Odometry.h>          ///< 里程计消息类型
#include <nav_msgs/Path.h>              ///< 路径消息类型
#include <std_msgs/Bool.h>              ///< 布尔消息类型
#include <std_msgs/Float32.h>           ///< 浮点数消息类型

// ==================== PCL相关头文件 ====================
#include <pcl_conversions/pcl_conversions.h> ///< PCL与ROS消息转换
#include <pcl/point_cloud.h>                 ///< PCL点云基础类
#include <pcl/point_types.h>                 ///< PCL点类型定义
#include <pcl/filters/voxel_grid.h>          ///< 体素网格滤波器
#include <pcl/io/pcd_io.h>                   ///< PCD文件读写
#include <pcl/common/transforms.h>           ///< 点云变换
#include <pcl/filters/crop_box.h>            ///< 立方体裁剪滤波器
#include <pcl/filters/passthrough.h>         ///< 直通滤波器

// ==================== TF变换相关头文件 ====================
#include <tf/LinearMath/Quaternion.h>        ///< TF四元数
#include <tf/transform_datatypes.h>          ///< TF数据类型转换
#include <tf2/LinearMath/Quaternion.h>       ///< TF2四元数
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> ///< TF2几何消息转换

// ==================== 数学库 ====================
#include <Eigen/Dense>                       ///< Eigen线性代数库

// ==================== ERASOR相关头文件 ====================
#include "erasor.h"                          ///< ERASOR核心算法
#include "tools/erasor_utils.hpp"            ///< ERASOR工具函数

/**
 * @brief 大型点云处理的点数阈值
 * @description 当点云数量超过此阈值时，认为是大规模点云数据
 */
#define NUM_PTS_LARGE_ENOUGH 200000

/**
 * @brief 地图点云处理的点数阈值
 * @description 地图级别点云数据的最大点数阈值
 */
#define NUM_PTS_LARGE_ENOUGH_FOR_MAP 20000000

namespace erasor {
    /**
     * @class OfflineMapUpdater
     * @brief ERASOR离线地图更新器类
     * @description 负责离线处理激光雷达数据，使用ERASOR算法去除动态对象，
     *              生成静态环境地图，支持多种数据格式和保存选项
     */
    class OfflineMapUpdater {
    private:
        // ==================== 核心算法参数 ====================
        
        /**< 从ROS参数服务器获取的MapUpdater参数 */
        double query_voxel_size_;           ///< 查询点云体素大小
        double map_voxel_size_;             ///< 地图点云体素大小
        
        /**< ERASOR不会在所有时间步都进行动态对象移除！
         * removal_interval需要一些启发式设置 */
        int    removal_interval_;           ///< 动态对象移除间隔
        int    global_voxelization_period_; ///< 全局体素化周期
        bool   verbose_;                    ///< 是否输出详细信息
        bool   is_large_scale_;             ///< 是否为大规模数据处理
        bool   is_submap_not_initialized_ = true; ///< 子地图是否未初始化

        // ==================== 感兴趣体积(VoI)参数 ====================
        
        /**< 感兴趣体积(Volume of Interest, VoI)参数 */
        double max_range_;                  ///< 最大检测距离
        double min_h_;                      ///< 最小高度阈值
        double max_h_;                      ///< 最大高度阈值
        double submap_size_;                ///< 子地图大小
        double submap_center_x_;            ///< 子地图中心X坐标
        double submap_center_y_;            ///< 子地图中心Y坐标

        // ==================== ERASOR算法版本和配置 ====================
        
        /**< ERASOR算法版本
         * v2: 朴素版本
         * v3: 更保守的处理方式*/
        int erasor_version_;                ///< ERASOR算法版本号

        // ==================== 数据集和路径配置 ====================
        
        std::string data_name_;             ///< 数据集名称
        std::string map_name_;              ///< 地图名称
        std::string environment_;           ///< 环境类型描述
        std::string save_path_;             ///< 结果保存路径

        unique_ptr<ERASOR> erasor_;         ///< ERASOR算法核心实例

        // ==================== ROS通信接口 ====================
        
        /**< 初始化相关参数 */
        int num_pcs_init_;                  ///< 初始化点云数量

        /**< ROS节点和通信组件 */
        ros::NodeHandle nh;                 ///< ROS节点句柄

        // 订阅器
        ros::Subscriber sub_node_;          ///< 节点数据订阅器
        ros::Subscriber sub_flag_;          ///< 标志位订阅器

        // 路径发布器
        ros::Publisher pub_path_;           ///< 路径轨迹发布器

        // 主要地图发布器
        ros::Publisher pub_map_init_;       ///< 初始地图发布器
        ros::Publisher pub_static_arranged_, pub_dynamic_arranged_; ///< 静态和动态排列地图发布器
        ros::Publisher pub_map_rejected_;   ///< 被拒绝地图点发布器
        ros::Publisher pub_curr_rejected_;  ///< 被拒绝当前点发布器

        // 调试发布器（用于算法开发和可视化）
        ros::Publisher pub_debug_map_arranged_init_;  ///< 调试：初始排列地图发布器
        ros::Publisher pub_debug_query_egocentric_;   ///< 调试：查询自中心点云发布器
        ros::Publisher pub_debug_map_egocentric_;     ///< 调试：地图自中心点云发布器

        ros::Publisher pub_debug_pc2_curr_; ///< 调试：当前点云发布器
        ros::Publisher pub_debug_map_;      ///< 调试：地图发布器

        // ==================== 点云数据指针 ====================
        
        /**< 全局地图相关变量 */
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_init_;              ///< 初始地图点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_;          ///< 排列后的地图点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_init_;     ///< 初始排列地图点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_global_;   ///< 全局排列地图点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_complement_; ///< 地图排列补充点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_ceilings_;          ///< 天花板点云

        /**< ERASOR算法输入点云 */
        pcl::PointCloud<pcl::PointXYZI>::Ptr query_voi_;             ///< 查询感兴趣区域点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi_;               ///< 地图感兴趣区域点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi_wrt_origin_;    ///< 相对于原点的地图感兴趣区域（地图坐标系）
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_outskirts_;         ///< 地图外围区域点云

        /**< ERASOR算法输出点云
         * 注意：map_filtered_ = map_static_estimate + map_egocentric_complement
         */
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_static_estimate_;   ///< 静态环境估计点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_egocentric_complement_; ///< 自中心补充点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered_;          ///< 滤波后的最终地图点云

        /**< 被拒绝的点云数据 */
        pcl::PointCloud<pcl::PointXYZI>::Ptr query_rejected_;        ///< 被拒绝的查询点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_rejected_;          ///< 被拒绝的地图点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr total_query_rejected_;  ///< 总的被拒绝查询点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr total_map_rejected_;    ///< 总的被拒绝地图点云

        /**< 可视化用点云 */
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_objs_to_viz_;   ///< 用于可视化的动态对象点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_objs_to_viz_;    ///< 用于可视化的静态对象点云

        // ==================== ROS消息数据 ====================
        
        /**< 发布的ROS消息 */
        sensor_msgs::PointCloud2 pc2_map_;                      ///< 地图点云消息
        nav_msgs::Path           path_;                         ///< 路径轨迹消息
        
        // ==================== 状态标志位 ====================
        
        /**< 系统状态标志 */
        bool is_map_init_;                                       ///< 地图是否已初始化
        bool is_map_updated_;                                    ///< 地图是否已更新
        bool is_save_flag_;                                      ///< 保存标志位
        
        // ==================== 计数器 ====================
        
        /**< 帧处理计数器 */
        int frame_idx_;                                          ///< 当前帧索引

        // ==================== 坐标变换矩阵 ====================
        
        /**< 坐标系变换矩阵 */
        Eigen::Matrix4f tf_lidar2body_;                         ///< 激光雷达到机体坐标系变换矩阵
        Eigen::Matrix4f tf_body2origin_;                        ///< 机体到原点坐标系变换矩阵

        // ==================== 私有成员函数 ====================
        
        /**
         * @brief 设置参数
         * @description 从启动文件加载参数配置
         */
        void set_params();

        /**
         * @brief 初始化点云指针
         * @description 为所有点云数据分配内存空间
         */
        void initialize_ptrs();

        /**
         * @brief 加载全局地图
         * @description 从文件加载预构建的全局地图数据
         */
        void load_global_map();

        /**
         * @brief 节点数据回调函数
         * @param msg 节点消息指针
         * @description 处理新的激光雷达数据节点
         */
        void callback_node(const erasor::node::ConstPtr &msg);

        /**
         * @brief 标志位回调函数
         * @param msg 浮点数标志消息指针
         * @description 用于保存结果点云到PCD文件的标志处理
         */
        void callback_flag(const std_msgs::Float32::ConstPtr &msg);

        /**
         * @brief 机体坐标系到原点坐标系变换
         * @param src 源点云
         * @param dst 目标点云
         * @description 将点云从机体坐标系变换到原点坐标系
         */
        void body2origin(
                const pcl::PointCloud<pcl::PointXYZI> src,
                pcl::PointCloud<pcl::PointXYZI> &dst);

        /**
         * @brief 重新分配子地图
         * @param pose_x 位姿X坐标
         * @param pose_y 位姿Y坐标
         * @description 根据当前位置重新分配感兴趣的子地图区域
         */
        void reassign_submap(double pose_x, double pose_y);

        /**
         * @brief 设置子地图
         * @param map_global 全局地图点云
         * @param submap 子地图点云
         * @param submap_complement 子地图补充点云
         * @param x 中心X坐标
         * @param y 中心Y坐标
         * @param submap_size 子地图大小
         * @description 从全局地图中提取指定区域的子地图
         */
        void set_submap(
                const pcl::PointCloud<pcl::PointXYZI> &map_global,
                pcl::PointCloud<pcl::PointXYZI>& submap,
                pcl::PointCloud<pcl::PointXYZI>& submap_complement,
                double x, double y, double submap_size);

        /**
         * @brief 提取感兴趣体积
         * @param x_criterion X方向判断标准
         * @param y_criterion Y方向判断标准
         * @param dst 目标感兴趣区域点云
         * @param outskirts 外围区域点云
         * @param mode 提取模式，默认为"naive"
         * @description 从地图中提取感兴趣的体积区域
         */
        void fetch_VoI(
                double x_criterion, double y_criterion,
                pcl::PointCloud<pcl::PointXYZI> &dst, pcl::PointCloud<pcl::PointXYZI> &outskirts,
                std::string mode = "naive");

        /**
         * @brief 打印状态信息
         * @description 输出当前处理状态和统计信息
         */
        void print_status();

        /**
         * @brief 设置路径
         * @param path 路径消息
         * @param mode 路径模式
         * @param node 节点数据
         * @param body2mapprev 机体到前一地图的变换矩阵
         * @description 设置机器人运动轨迹路径
         */
        void set_path(
                nav_msgs::Path &path, std::string mode,
                const erasor::node &node, const Eigen::Matrix4f &body2mapprev);

        /**
         * @brief 发布点云消息
         * @param map 点云消息
         * @param publisher 发布器
         * @description 发布sensor_msgs::PointCloud2格式的点云数据
         */
        void publish(
                const sensor_msgs::PointCloud2 &map,
                const ros::Publisher &publisher);

        /**
         * @brief 发布PCL点云
         * @param map PCL点云
         * @param publisher 发布器
         * @description 发布pcl::PointCloud<pcl::PointXYZI>格式的点云数据
         */
        void publish(
                const pcl::PointCloud<pcl::PointXYZI> &map,
                const ros::Publisher &publisher);

        /**< 当前位姿 */
        geometry_msgs::Pose pose_curr;                          ///< 当前机器人位姿

    public:
        // ==================== 公共接口方法 ====================
        
        /**
         * @brief 构造函数
         * @description 初始化离线地图更新器，设置默认参数
         */
        OfflineMapUpdater();

        /**
         * @brief 析构函数
         * @description 清理资源，释放内存
         */
        ~OfflineMapUpdater();

        /**
         * @brief 保存静态地图
         * @param voxel_size 体素大小
         * @description 将处理后的静态地图以指定体素大小保存到文件
         */
        void save_static_map(float voxel_size);
    };

}


#endif

