/**
 * @file erasor_utils.cpp
 * @brief ERASOR工具函数实现文件
 * @author ERASOR团队
 * @description 提供ERASOR算法所需的各种工具函数，包括坐标变换、点云处理、动态对象分类等功能
 */

#include "tools/erasor_utils.hpp"

// 动态对象类别标签定义（基于SemanticKITTI数据集）
// 包含车辆、行人、自行车等移动对象的语义标签
std::vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};

/**
 * @namespace erasor_utils
 * @brief ERASOR工具函数命名空间
 * @description 包含坐标变换、点云处理、动态对象检测等核心工具函数
 */
namespace erasor_utils {
    /**
     * @brief 将Eigen变换矩阵转换为ROS几何位姿
     * @param pose 4x4变换矩阵（Eigen::Matrix4f格式）
     * @return geometry_msgs::Pose ROS标准位姿消息
     * @description 提取变换矩阵中的旋转和平移信息，转换为ROS位姿格式
     */
    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose) {
        geometry_msgs::Pose geoPose;

        // 提取旋转矩阵部分（3x3）
        tf::Matrix3x3 m;
        m.setValue((double) pose(0, 0),
                   (double) pose(0, 1),
                   (double) pose(0, 2),
                   (double) pose(1, 0),
                   (double) pose(1, 1),
                   (double) pose(1, 2),
                   (double) pose(2, 0),
                   (double) pose(2, 1),
                   (double) pose(2, 2));

        // 将旋转矩阵转换为四元数
        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        // 提取平移向量部分
        geoPose.position.x = pose(0, 3);  ///< X轴平移量
        geoPose.position.y = pose(1, 3);  ///< Y轴平移量
        geoPose.position.z = pose(2, 3);  ///< Z轴平移量

        return geoPose;
    }

    /**
     * @brief 将ROS几何位姿转换为Eigen变换矩阵
     * @param geoPose ROS几何位姿消息
     * @return Eigen::Matrix4f 4x4变换矩阵
     * @description 从ROS位姿消息中提取旋转和平移信息，构建Eigen变换矩阵
     */
    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose) {
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();  ///< 初始化为4x4单位矩阵
        
        // ==================== 四元数转旋转矩阵 ====================
        tf::Quaternion  q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w); ///< 构建TF四元数
        tf::Matrix3x3   m(q);  ///< 从四元数构建3x3旋转矩阵
        
        // ==================== 填充旋转矩阵部分（左上角3x3） ====================
        result(0, 0) = m[0][0];  ///< R11
        result(0, 1) = m[0][1];  ///< R12
        result(0, 2) = m[0][2];  ///< R13
        result(1, 0) = m[1][0];  ///< R21
        result(1, 1) = m[1][1];  ///< R22
        result(1, 2) = m[1][2];  ///< R23
        result(2, 0) = m[2][0];  ///< R31
        result(2, 1) = m[2][1];  ///< R32
        result(2, 2) = m[2][2];  ///< R33
        result(3, 3) = 1;        ///< 齐次坐标系标识

        // ==================== 填充平移向量部分（右上角3x1） ====================
        result(0, 3) = geoPose.position.x;  ///< X轴平移量
        result(1, 3) = geoPose.position.y;  ///< Y轴平移量
        result(2, 3) = geoPose.position.z;  ///< Z轴平移量

        return result;
    }

    /**
     * @brief 解析点云中的动态和静态对象
     * @param cloudIn 输入点云（包含语义标签信息）
     * @param dynamicOut 输出的动态对象点云
     * @param staticOut 输出的静态对象点云
     * @description 根据点云强度值中编码的语义标签，将点云分类为动态和静态对象
     *              强度值的低16位为语义标签，高16位为实例标签
     */
    void parse_dynamic_obj(
            const pcl::PointCloud<pcl::PointXYZI> &cloudIn, pcl::PointCloud<pcl::PointXYZI> &dynamicOut,
            pcl::PointCloud<pcl::PointXYZI> &staticOut) {
        // ==================== 初始化输出点云 ====================
        dynamicOut.points.clear();  ///< 清空动态对象点云容器
        staticOut.points.clear();   ///< 清空静态对象点云容器

        // ==================== 遍历处理每个点 ====================
        for (const auto &pt: cloudIn.points) {
            // ==================== 解析语义标签信息 ====================
            uint32_t float2int      = static_cast<uint32_t>(pt.intensity);  ///< 将浮点强度值转换为32位整数
            uint32_t semantic_label = float2int & 0xFFFF;                   ///< 提取语义标签（低16位）
            uint32_t inst_label     = float2int >> 16;                      ///< 提取实例标签（高16位）
            bool     is_static      = true;                                 ///< 默认假设为静态对象
            
            // ==================== 动态对象分类检测 ====================
            for (int class_num: DYNAMIC_CLASSES) {
                if (semantic_label == class_num) {  ///< 检查是否属于预定义的动态对象类别
                    dynamicOut.points.push_back(pt); ///< 添加到动态对象点云
                    is_static = false;               ///< 标记为动态对象
                    break;                           ///< 找到匹配类别后退出循环
                }
            }
            
            // ==================== 静态对象归类 ====================
            if (is_static) {
                staticOut.points.push_back(pt);  ///< 将非动态对象归类为静态对象
            }
        }
    }

    /**
     * @brief 保留语义标签的点云体素化处理
     * @param src 输入点云指针
     * @param dst 输出体素化后的点云
     * @param leaf_size 体素网格的叶子大小
     * @description 由于PCL的体素化会对强度值进行平均处理，导致语义标签信息丢失，
     *              本函数先进行体素化，然后通过最近邻搜索重新分配原始标签信息
     */
    void voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size) {
        /**< 重要说明：
         * 由于PCL体素化只是对点云强度进行平均处理，
         * 因此本函数先进行体素化，然后通过最近邻点搜索重新分配每个点的标签 */
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZI>);

        // 1. 体素化处理
        static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(src);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*ptr_voxelized);

        // 2. 查找最近邻点以更新强度值（索引和标签）
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(src);  // 设置KD树的输入点云

        ptr_reassigned->points.reserve(ptr_voxelized->points.size());  // 预分配内存

        int K = 1;  // 查找1个最近邻点

        std::vector<int>   pointIdxNKNSearch(K);        // 最近邻点索引
        std::vector<float> pointNKNSquaredDistance(K);  // 最近邻点距离平方

        // 为体素化后的每个点重新分配原始标签
        for (const auto &pt: ptr_voxelized->points) {
            // 在原始点云中查找最近邻点
            if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                auto updated = pt;
                // 将平均后的强度值更新为原始强度值（保留语义标签）
                updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
                ptr_reassigned->points.emplace_back(updated);
            }
        }
        dst = *ptr_reassigned;  // 返回重新分配标签后的点云
    }

    /**
     * @brief 统计点云中静态和动态对象的数量
     * @param cloudIn 输入点云（包含语义标签信息）
     * @param num_static 输出的静态对象点数
     * @param num_dynamic 输出的动态对象点数
     * @description 根据点云强度值中的语义标签，统计静态和动态对象的点数
     */
    void count_stat_dyn(const pcl::PointCloud<pcl::PointXYZI> &cloudIn, int &num_static, int &num_dynamic) {
        int             tmp_static  = 0;   // 临时静态点计数器
        int             tmp_dynamic = 0;   // 临时动态点计数器
        
        // 遍历输入点云中的每个点
        for (const auto &pt: cloudIn.points) {
            // 解析强度值中的语义标签信息
            uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
            uint32_t semantic_label = float2int & 0xFFFF;  // 提取语义标签（低16位）
            uint32_t inst_label     = float2int >> 16;     // 提取实例标签（高16位）
            bool     is_static      = true;
            
            // 检查是否属于动态对象类别
            for (int class_num: DYNAMIC_CLASSES) {
                if (semantic_label == class_num) { // 检查是否在移动对象类别中
                    is_static = false;
                    break;
                }
            }
            
            // ==================== 根据分类结果更新计数器 ====================
            if (is_static) {
                tmp_static++;   ///< 静态对象点数加1
            } else {
                tmp_dynamic++;  ///< 动态对象点数加1
            }
        }
        
        // ==================== 返回统计结果 ====================
        num_static  = tmp_static;   ///< 输出静态对象总点数
        num_dynamic = tmp_dynamic;  ///< 输出动态对象总点数
    }

    /**
     * @brief 信号回调处理函数
     * @param signum 信号编号（如SIGINT=2表示Ctrl+C）
     * @description 处理系统中断信号，确保程序能够优雅地退出
     *              主要用于处理用户按下Ctrl+C时的清理工作
     */
    void signal_callback_handler(int signum) {
        cout << "Caught Ctrl + c " << endl;  ///< 输出中断信号捕获提示
        exit(signum);  ///< 以信号编号作为退出码退出程序
    }
}