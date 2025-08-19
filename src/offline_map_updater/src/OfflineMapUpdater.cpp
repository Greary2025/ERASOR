/**
 * @file OfflineMapUpdater.cpp
 * @brief 离线地图更新器实现文件
 * @author ERASOR团队
 * @description 实现离线地图更新功能，使用ERASOR算法从激光雷达地图中移除动态对象
 *              支持大规模地图处理和室内外环境适配
 */

#include "erasor/OfflineMapUpdater.h"

using namespace erasor;

/**
 * @brief OfflineMapUpdater类构造函数
 * @description 初始化ROS订阅者、发布者，设置参数，加载全局地图，创建ERASOR实例
 */
OfflineMapUpdater::OfflineMapUpdater() {
    // 订阅优化后的节点信息
    sub_node_ = nh.subscribe<erasor::node>("/node/combined/optimized", 2000, &OfflineMapUpdater::callback_node, this);
    // 订阅保存标志
    sub_flag_ = nh.subscribe<std_msgs::Float32>("/saveflag", 10, &OfflineMapUpdater::callback_flag, this);

    // 发布修正后的路径
    pub_path_ = nh.advertise<nav_msgs::Path>("/MapUpdater/path_corrected", 100);

    // 发布地图相关点云
    pub_map_init_      = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_init", 100);      // 初始地图
    pub_map_rejected_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_rejected", 100);  // 地图中被拒绝的点
    pub_curr_rejected_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/curr_rejected", 100); // 当前帧被拒绝的点

    // 发布调试用当前点云
    pub_debug_pc2_curr_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/pc2_curr", 100);

    // 发布调试用点云数据
    pub_debug_map_               = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map", 100);                // 调试地图
    pub_debug_query_egocentric_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/pc_curr_body", 100);       // 自车坐标系下的查询点云
    pub_debug_map_egocentric_    = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_body", 100);           // 自车坐标系下的地图点云
    pub_debug_map_arranged_init_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_init_arranged", 100);  // 初始整理后的地图

    // 发布动态和静态对象点云
    pub_dynamic_arranged_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/dynamic", 100);  // 动态对象点云
    pub_static_arranged_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/static", 100);   // 静态对象点云

    initialize_ptrs();  // 初始化智能指针

    set_params();       // 设置参数

    load_global_map();  // 加载全局地图

    erasor_.reset(new ERASOR(&nh));  // 创建ERASOR算法实例
}

/**
 * @brief OfflineMapUpdater类析构函数
 * @description 清理OfflineMapUpdater实例资源，释放内存空间
 */
OfflineMapUpdater::~OfflineMapUpdater() {
}

/**
 * @brief 初始化所有点云智能指针
 * @description 为各种用途的点云数据分配内存，包括地图、查询、调试和可视化点云
 *              初始化地图相关、感兴趣区域相关、处理结果相关、被拒绝点云和可视化用点云
 */
void OfflineMapUpdater::initialize_ptrs() {
    // ==================== 地图相关点云 ====================
    map_init_.reset(new pcl::PointCloud<pcl::PointXYZI>());                ///< 原始初始地图点云
    map_arranged_.reset(new pcl::PointCloud<pcl::PointXYZI>());            ///< ERASOR处理后的整理地图
    map_arranged_init_.reset(new pcl::PointCloud<pcl::PointXYZI>());       ///< 初始整理地图（用于比较）
    map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());     ///< 全局累积的整理地图
    map_arranged_complement_.reset(new pcl::PointCloud<pcl::PointXYZI>()); ///< 地图补充点云（超出范围的点）
    map_ceilings_.reset(new pcl::PointCloud<pcl::PointXYZI>());            ///< 天花板点云（室内环境专用）

    // ==================== 感兴趣区域相关点云 ====================
    query_voi_.reset(new pcl::PointCloud<pcl::PointXYZI>());        ///< 查询感兴趣区域点云（当前帧）
    map_voi_.reset(new pcl::PointCloud<pcl::PointXYZI>());          ///< 地图感兴趣区域点云
    map_voi_wrt_origin_.reset(new pcl::PointCloud<pcl::PointXYZI>());///< 相对于原点的地图感兴趣区域
    map_outskirts_.reset(new pcl::PointCloud<pcl::PointXYZI>());    ///< 地图外围区域点云

    // ==================== 处理结果相关点云 ====================
    map_static_estimate_.reset(new pcl::PointCloud<pcl::PointXYZI>());     ///< ERASOR算法估计的静态地图
    map_egocentric_complement_.reset(new pcl::PointCloud<pcl::PointXYZI>());///< 自车坐标系下的补充地图
    map_filtered_.reset(new pcl::PointCloud<pcl::PointXYZI>());            ///< 经过滤波处理的地图

    // ==================== 被拒绝点云（动态对象） ====================
    query_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());       ///< 当前帧中被识别为动态的点
    map_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());         ///< 地图中被识别为动态的点
    total_query_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>()); ///< 累积的查询被拒绝点
    total_map_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());   ///< 累积的地图被拒绝点

    // ==================== 可视化用点云 ====================
    dynamic_objs_to_viz_.reset(new pcl::PointCloud<pcl::PointXYZI>());  ///< 用于RViz显示的动态对象点云
    static_objs_to_viz_.reset(new pcl::PointCloud<pcl::PointXYZI>());   ///< 用于RViz显示的静态对象点云
}

/**
 * @brief 设置系统参数
 * @description 从ROS参数服务器读取各种配置参数，包括体素化、地图处理、ERASOR算法等参数
 *              同时设置激光雷达到车体的坐标变换
 */
void OfflineMapUpdater::set_params() {
    nh = ros::NodeHandle("~");
    
    // OfflineMapUpdater 参数
    nh.param("/MapUpdater/query_voxel_size", query_voxel_size_, 0.05);              // 查询点云体素大小
    nh.param("/MapUpdater/map_voxel_size", map_voxel_size_, 0.05);                  // 地图点云体素大小
    nh.param("/MapUpdater/voxelization_interval", global_voxelization_period_, 10); // 全局体素化周期
    nh.param("/MapUpdater/removal_interval", removal_interval_, 2);                 // 移除间隔
    nh.param<std::string>("/MapUpdater/data_name", data_name_, "00");               // 数据集名称
    nh.param<std::string>("/MapUpdater/env", environment_, "outdoor");             // 环境类型（室内/室外）
    nh.param<std::string>("/MapUpdater/initial_map_path", map_name_, "/");         // 初始地图路径
    nh.param<std::string>("/MapUpdater/save_path", save_path_, "/");               // 保存路径

    // 大规模地图处理参数
    nh.param<bool>("/large_scale/is_large_scale", is_large_scale_, false);         // 是否为大规模地图
    nh.param("/large_scale/submap_size", submap_size_, 200.0);                     // 子地图大小

    // ERASOR算法参数
    nh.param("/erasor/max_range", max_range_, 60.0);    // 最大检测距离
    nh.param("/erasor/max_h", max_h_, 3.0);             // 最大高度
    nh.param("/erasor/min_h", min_h_, 0.0);             // 最小高度
    nh.param("/erasor/version", erasor_version_, 3);    // ERASOR版本

    // 调试输出参数
    nh.param("/verbose", verbose_, true);
    std::cout << "Loading " << map_name_ << endl;
    std::cout << "Target env: " << environment_ << std::endl;
    std::cout << "\033[1;32m Version: \033[0m: " << erasor_version_ << std::endl;
    
    // 设置激光雷达到车体的坐标变换
    std::vector<double> lidar2body;
    if (nh.getParam("/tf/lidar2body", lidar2body)) {
        if (lidar2body.size() == 7) {
            geometry_msgs::Pose tmp_pose;
            tmp_pose.position.x    = lidar2body[0];     // X位置
            tmp_pose.position.y    = lidar2body[1];     // Y位置
            tmp_pose.position.z    = lidar2body[2];     // Z位置
            tmp_pose.orientation.x = lidar2body[3];     // 四元数X
            tmp_pose.orientation.y = lidar2body[4];     // 四元数Y
            tmp_pose.orientation.z = lidar2body[5];     // 四元数Z
            tmp_pose.orientation.w = lidar2body[6];     // 四元数W
            Eigen::Matrix4f tmp_tf = Eigen::Matrix4f::Identity();
            tf_lidar2body_ = erasor_utils::geoPose2eigen(tmp_pose) * tmp_tf;
            std::cout << tf_lidar2body_ << std::endl;
            tmp_pose = erasor_utils::eigen2geoPose(tf_lidar2body_);
        }
    }
}

/**
 * @brief 加载全局地图
 * @description 加载初始地图文件并根据环境类型进行预处理
 *              map_init_: 原始初始地图
 *              map_arranged_init_: 用于与整理地图比较的初始地图
 *                                  室内环境: map_arranged_init_ = map_init_ - 天花板
 *                                  室外环境: map_arranged_init_ = map_init_
 *              map_arranged_: 通过ERASOR算法过滤的目标点云
 */
void OfflineMapUpdater::load_global_map() {
    cout<<"[MapUpdater] 正在加载初始累积地图...这需要几秒钟..."<<endl;
    
    // 为各种地图点云预分配足够的内存空间
    map_init_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    map_arranged_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    map_arranged_init_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    map_arranged_global_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

    // 加载PCD格式的地图文件
    int failure_flag = erasor_utils::load_pcd(map_name_, map_init_);

    if (failure_flag == -1) {
        throw invalid_argument("初始地图路径可能不正确!");
    } else {
        std::cout << "全局地图加载完成!" << std::endl;
    }

    num_pcs_init_ = map_init_->points.size();  // 记录初始点云数量

    if (environment_ == "outdoor") {
        // 室外环境：直接使用原始地图
        *map_arranged_      = *map_init_;
        *map_arranged_init_ = *map_arranged_;
        
        if (is_large_scale_) {
            // 大规模静态地图构建模式
            // map_arranged_ 用作子地图
            // 之后 map_arranged_ 会更新到 map_arranged_global_
            *map_arranged_global_ = *map_arranged_;
            std::cout << "大规模模式已开启!" << std::endl;
            std::cout << "子地图大小为: " << submap_size_ << std::endl;
        }

    } else if (environment_ == "indoor") {
        // 室内环境：需要移除天花板
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_dst(new pcl::PointCloud<pcl::PointXYZI>);
        // TODO: 此功能可能在大规模地图或包含斜坡区域的环境中无法正常工作
        throw invalid_argument("室内模式尚不完善!");
        pcl::PassThrough<pcl::PointXYZI> ptfilter;
        ptfilter.setInputCloud(map_init_);
        ptfilter.setFilterFieldName("z");
        ptfilter.setFilterLimits(min_h_, max_h_);
        ptfilter.filter(*ptr_dst);

        *map_arranged_init_ = *map_init_;
        *map_arranged_      = *map_arranged_init_;

        ptfilter.setFilterLimitsNegative(true);
        ptfilter.filter(*ptr_dst);
        *map_ceilings_ = *ptr_dst;
    }
    if (!is_large_scale_) {
        // 大规模模式下发布会耗费太多时间
        pub_map_init_.publish(erasor_utils::cloud2msg(*map_arranged_init_));
    }
}

/**
 * @brief 标志回调函数
 * @param msg 包含体素大小的标志消息
 * @description 接收保存地图的标志信号，触发静态地图保存
 */
void OfflineMapUpdater::callback_flag(const std_msgs::Float32::ConstPtr &msg) {
    std::cout << "接收到保存标志!" << std::endl;
    save_static_map(msg->data);
}

/**
 * @brief 保存静态地图
 * @param voxel_size 体素化大小
 * @description 对处理后的地图进行体素化并保存为PCD文件
 */
void OfflineMapUpdater::save_static_map(float voxel_size) {
    // 1. 体素化处理
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    ptr_src->reserve(num_pcs_init_);

    if (is_large_scale_) {
        std::cout << "合并子地图和补充点云..." << std::endl;
        *ptr_src = *map_arranged_ + *map_arranged_complement_;
    } else {
        *ptr_src = *map_arranged_;
    }
    pcl::PointCloud<pcl::PointXYZI> map_to_be_saved;
    erasor_utils::voxelize_preserving_labels(ptr_src, map_to_be_saved, voxel_size); // 0.05m为标准体素大小
    
    // 2. 保存点云地图
    map_to_be_saved.width  = map_to_be_saved.points.size();
    map_to_be_saved.height = 1;

    std::cout << "\033[1;32m目标文件: " << save_path_ + "/" + data_name_ + "_result.pcd" << "\033[0m" << std::endl;
    std::cout << "使用体素大小 " << voxel_size << " 进行体素化" << std::endl;
    pcl::io::savePCDFileASCII(save_path_ + "/" + data_name_ + "_result.pcd", map_to_be_saved);
    std::cout << "\033[1;32m最终静态地图保存完成\033[0m" << std::endl;

}



/**
 * @brief 节点数据回调函数
 * @param msg 包含激光雷达数据和里程计信息的节点消息
 * @description 处理传入的节点数据，执行ERASOR算法进行动态对象移除
 */
void OfflineMapUpdater::callback_node(const erasor::node::ConstPtr &msg) {
    signal(SIGINT, erasor_utils::signal_callback_handler);  // 设置中断信号处理

    static int stack_count = 0;
    stack_count++;

    // 按照移除间隔处理数据
    if (stack_count % removal_interval_ == 0) {
        if (verbose_) ROS_INFO_STREAM("\033[01;32m第" << msg->header.seq << "帧\033[0m 正在处理");

        if (!is_large_scale_) {
            pub_debug_map_arranged_init_.publish(erasor_utils::cloud2msg(*map_arranged_init_));
        }
        
        // 假设：传入的节点与用于构建初始地图的节点相同
        // 前提：节点已经过某种程度的优化
        tf_body2origin_ = erasor_utils::geoPose2eigen(msg->odom);
        set_path(path_, "corrected", *msg, tf_body2origin_);

        // 1. 设置查询点云 -> 查询感兴趣区域(VOI)
        // 注意：查询点云在激光雷达坐标系下
        // 因此需要从激光雷达坐标系转换到车体坐标系（具体取决于环境设置）
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_voxel(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_body(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_viz(new pcl::PointCloud<pcl::PointXYZI>);

        // 为点云预分配内存
        ptr_query->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_voxel->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_body->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_viz->reserve(NUM_PTS_LARGE_ENOUGH);

        // 从ROS消息转换为PCL点云
        pcl::fromROSMsg(msg->lidar, *ptr_query);
        // 体素化处理，保留标签信息
        erasor_utils::voxelize_preserving_labels(ptr_query, *ptr_query_voxel, query_voxel_size_);

        // 坐标变换：从激光雷达坐标系到车体坐标系
        pcl::transformPointCloud(*ptr_query_voxel, *ptr_query_body, tf_lidar2body_);
        *query_voi_ = *ptr_query_body;
        body2origin(*ptr_query_body, *ptr_query_viz);  // 转换到全局坐标系用于可视化
        // - - - - - - - - - - - - - - - - - - - -

        // 2. 设置地图点云 -> 地图感兴趣区域(VOI)
        double x_curr = tf_body2origin_(0, 3);  // 当前X位置
        double y_curr = tf_body2origin_(1, 3);  // 当前Y位置

        if (is_large_scale_) {
            reassign_submap(x_curr, y_curr);  // 大规模模式下重新分配子地图
        }

        auto start_voi = ros::Time::now().toSec();  // 记录VOI处理开始时间
        fetch_VoI(x_curr, y_curr, *map_voi_, *map_outskirts_);  // 提取感兴趣区域
        auto end_voi = ros::Time::now().toSec();

        ROS_INFO_STREAM("\033[1;32m" << setw(22) << "提取VOI耗时 " << end_voi - start_voi << "s\033[0m");

        // 发布调试信息：自车中心的地图和查询点云
        pub_debug_map_egocentric_.publish(erasor_utils::cloud2msg(*map_voi_));
        pub_debug_query_egocentric_.publish(erasor_utils::cloud2msg(*query_voi_));

        // 3. 执行扫描比率测试并设置静态估计地图和自车中心补充地图
        // 注意：输入应该已经转换到自车中心坐标系
        auto start = ros::Time::now().toSec();

        erasor_->set_inputs(*map_voi_, *query_voi_);  // 设置ERASOR算法输入
        if (erasor_version_ == 2) {
            erasor_->compare_vois_and_revert_ground(msg->header.seq);
            erasor_->get_static_estimate(*map_static_estimate_, *map_egocentric_complement_);
        } else if (erasor_version_ == 3) {
            erasor_->compare_vois_and_revert_ground_w_block(msg->header.seq);
            erasor_->get_static_estimate(*map_static_estimate_, *map_egocentric_complement_);
        } else {
            throw invalid_argument("其他版本尚未实现!");
        }

        auto middle = ros::Time::now().toSec();

        ROS_INFO_STREAM("\033[1;32m" << setw(22) << "ERASOR算法耗时 " << middle - start << "s\033[0m");

        // 合并静态估计和补充点云
        *map_filtered_ = *map_static_estimate_ + *map_egocentric_complement_;

        // 获取当前被拒绝的点
        erasor_->get_outliers(*map_rejected_, *query_rejected_);

        // 将结果转换回全局坐标系
        body2origin(*map_filtered_, *map_filtered_);
        body2origin(*map_rejected_, *map_rejected_);
        body2origin(*query_rejected_, *query_rejected_);

        // 合并过滤后的地图和外围区域
        *map_arranged_ = *map_filtered_ + *map_outskirts_;

        auto end = ros::Time::now().toSec();

        // 解析动态和静态对象用于可视化
        erasor_utils::parse_dynamic_obj(*map_arranged_, *dynamic_objs_to_viz_, *static_objs_to_viz_);

        // 调试用：累积被拒绝的点
        *total_map_rejected_ += *map_rejected_;
        *total_query_rejected_ += *query_rejected_;

        // 全局体素化（已注释）
        // if (stack_count % global_voxelization_period_ == 0) { // 1表示初始体素化
        //     pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
        //     *ptr_src = *map_arranged_;
        //     auto num_origin  = map_arranged_->points.size();
        //     auto start_voxel = ros::Time::now().toSec();
        //     erasor_utils::voxelize_preserving_labels(ptr_src, *map_arranged_, map_voxel_size_);
        //     auto end_voxel = ros::Time::now().toSec();
        //     map_arranged_->width  = map_arranged_->points.size();
        //     map_arranged_->height = 1;
        //     ROS_INFO_STREAM("全局体素化操作: " << setw(10) << num_origin << " -> " << map_arranged_->points.size());
        //     ROS_INFO_STREAM(setw(22) <<"全局体素化耗时 " << end_voxel - start_voxel << "s");
        // }
        
        if (environment_ != "outdoor") { throw invalid_argument("不支持其他模式"); }

        print_status();  // 打印状态信息

        // 发布各种点云用于可视化和调试
        publish(*ptr_query_viz, pub_debug_pc2_curr_);          // 当前查询点云
        publish(*static_objs_to_viz_, pub_static_arranged_);    // 静态对象
        publish(*dynamic_objs_to_viz_, pub_dynamic_arranged_);  // 动态对象
        publish(*map_rejected_, pub_map_rejected_);             // 被拒绝的地图点
        publish(*query_rejected_, pub_curr_rejected_);          // 被拒绝的查询点

        if (!is_large_scale_) {
            publish(*map_init_, pub_map_init_);  // 初始地图（非大规模模式）
        }

        pub_path_.publish(path_);  // 发布路径
    } else {
        ROS_INFO_STREAM("\033[1;32m 跳过处理! \033[0m");
    }
}

/**
 * @brief 重新分配子地图
 * @param pose_x 当前位置X坐标
 * @param pose_y 当前位置Y坐标
 * @description 根据当前位置动态调整子地图范围，用于大规模地图处理
 */
void OfflineMapUpdater::reassign_submap(double pose_x, double pose_y){
    if (is_submap_not_initialized_) {
        // 首次初始化子地图
        set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
        submap_center_x_ = pose_x;
        submap_center_y_ = pose_y;
        is_submap_not_initialized_ = false;

        ROS_INFO_STREAM("\033[1;32m子地图初始化完成!\033[0m");
        ROS_INFO_STREAM(map_arranged_global_->points.size() <<" 到 " << map_arranged_->points.size() <<" | " <<map_arranged_complement_->points.size());
    } else {
        // 检查是否需要重新分配子地图
        double diff_x = abs(submap_center_x_ - pose_x);
        double diff_y = abs(submap_center_y_ - pose_y);
        static double half_size = submap_size_ / 2.0;
        if ( (diff_x > half_size) ||  (diff_y > half_size) ) {
            // 重新分配子地图
            map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_arranged_global_->reserve(num_pcs_init_);
            *map_arranged_global_ = *map_arranged_ + *map_arranged_complement_;

            set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
            submap_center_x_ = pose_x;
            submap_center_y_ = pose_y;
            ROS_INFO_STREAM("\033[1;32m子地图重新分配完成!\033[0m");
            ROS_INFO_STREAM(map_arranged_global_->points.size() <<" 到 " << map_arranged_->points.size() <<" | " <<map_arranged_complement_->points.size());
        }
    }
}

/**
 * @brief 设置子地图
 * @param map_global 全局地图
 * @param submap 输出的子地图
 * @param submap_complement 输出的子地图补充部分
 * @param x 中心位置X坐标
 * @param y 中心位置Y坐标
 * @param submap_size 子地图大小
 * @description 根据指定中心位置和大小从全局地图中提取子地图
 */
void OfflineMapUpdater::set_submap(
        const pcl::PointCloud<pcl::PointXYZI> &map_global, pcl::PointCloud<pcl::PointXYZI>& submap,
        pcl::PointCloud<pcl::PointXYZI>& submap_complement,
        double x, double y, double submap_size) {

    // 清空并预分配内存
    submap.clear();
    submap.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    submap_complement.clear();
    submap_complement.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

    // 根据距离将点分配到子地图或补充地图
    for (const auto pt: map_global.points) {
        double diff_x = fabs(x - pt.x);  // X方向距离
        double diff_y = fabs(y - pt.y);  // Y方向距离
        if ((diff_x < submap_size) && (diff_y < submap_size)) {
            submap.points.emplace_back(pt);  // 在子地图范围内
        } else {
            submap_complement.points.emplace_back(pt);  // 在子地图范围外
        }
    }
}

/**
 * @brief 提取感兴趣区域(VoI)
 * @param x_criterion X方向判断标准
 * @param y_criterion Y方向判断标准
 * @param dst 输出的感兴趣区域点云
 * @param outskirts 输出的外围区域点云
 * @param mode 处理模式
 * @description 将地图分割为中心感兴趣区域和外围区域
 */
void OfflineMapUpdater::fetch_VoI(
        double x_criterion, double y_criterion, pcl::PointCloud<pcl::PointXYZI> &dst,
        pcl::PointCloud<pcl::PointXYZI> &outskirts, std::string mode) {
    // 1. 将map_arranged分割为中心地图和外围地图
    static double margin = 0;
    if (!dst.empty()) dst.clear();
    if (!outskirts.empty()) outskirts.clear();
    if (!map_voi_wrt_origin_->points.empty()) map_voi_wrt_origin_->points.clear(); // 内点仍在地图坐标系下

    if (mode == "naive") {
        // 朴素模式：基于欧几里得距离的简单分割
        double max_dist_square = pow(max_range_ + margin, 2);

        for (auto const &pt : (*map_arranged_).points) {
            double dist_square = pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
            if (dist_square < max_dist_square) {
                map_voi_wrt_origin_->points.emplace_back(pt);  // 在感兴趣区域内
            } else {
                outskirts.points.emplace_back(pt);  // 在外围区域
            }
        }
    } else if (mode == "kdtree") {
        // KD树模式：使用KD树进行高效的半径搜索
        pcl::PointXYZI searchPoint;
        searchPoint.x = x_criterion;
        searchPoint.y = y_criterion;  // 修正：应该是y而不是x
        searchPoint.z = 0.5;
        std::cout << "\033[1;32mKDTREE模式 " << (*map_arranged_).points.size() << "\033[0m" << std::endl;
        std::vector<int>                     pointIdxRadiusSearch;
        std::vector<float>                   pointRadiusSquaredDistance;
        pcl::KdTreeFLANN<pcl::PointXYZI>     kdtree;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        *cloud = *map_arranged_;
        kdtree.setInputCloud(cloud);

        if (kdtree.radiusSearch(searchPoint, max_range_ + 0.5, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // 获取外围点
            std::vector<char> isTrue(map_arranged_->points.size(), false);
            std::cout << "搜索到的点数: " << pointIdxRadiusSearch.size();
            std::cout << "    总点数: " << isTrue.size();
            for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                auto pt = (*cloud)[pointIdxRadiusSearch[i]];
                map_voi_wrt_origin_->points.emplace_back(pt);
                isTrue[pointIdxRadiusSearch[i]] = true;
            }
            for (size_t j = 0; j < map_arranged_->points.size(); ++j) {
                if (!isTrue[j]) {
                    outskirts.push_back(map_arranged_->points[j]);
                }
            }
        }
    }
    ROS_INFO_STREAM(map_arranged_->points.size() << "=" << map_voi_wrt_origin_->points.size() + outskirts.points.size() << "| "
                                                 << map_voi_wrt_origin_->points.size() << " + \033[4;32m" << outskirts.points.size()
                                                 << "\033[0m");

    // 将VOI从全局坐标系转换到车体坐标系
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*map_voi_wrt_origin_, *ptr_transformed, tf_body2origin_.inverse());
    dst = *ptr_transformed;
}


/**
 * @brief 车体坐标系到全局坐标系转换
 * @param src 输入点云（车体坐标系）
 * @param dst 输出点云（全局坐标系）
 * @description 将点云从车体坐标系转换到全局坐标系
 */
void OfflineMapUpdater::body2origin(
        const pcl::PointCloud<pcl::PointXYZI> src,
        pcl::PointCloud<pcl::PointXYZI> &dst) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    *ptr_src = src;
    pcl::transformPointCloud(*ptr_src, *ptr_transformed, tf_body2origin_);
    dst = *ptr_transformed;
}

/**
 * @brief 打印状态信息
 * @description 输出ERASOR算法处理的点云数量统计信息
 */
void OfflineMapUpdater::print_status() {
    ROS_INFO_STREAM("ERASOR输入: \033[1;33m" << map_voi_->points.size() << "\033[0m = ");
    ROS_INFO_STREAM(map_static_estimate_->points.size() << " + " << map_egocentric_complement_->points.size() << " - "
                                                        << map_rejected_->points.size());
    ROS_INFO_STREAM(" = \033[1;33m" << map_static_estimate_->points.size() + map_egocentric_complement_->points.size() -
                                       map_rejected_->points.size() << "\033[0m");
    ROS_INFO_STREAM(map_arranged_->points.size() << " " << map_filtered_->points.size() << " \033[4;32m"
                                                 << map_outskirts_->points.size() << "\033[0m");

    std::cout << "[Debug] Total: " << map_arranged_->points.size() << std::endl;
    std::cout << "[Debug] " << (double) dynamic_objs_to_viz_->points.size() / num_pcs_init_ * 100 << setw(7) << " % <- "
              << dynamic_objs_to_viz_->points.size() << " / " << num_pcs_init_ << std::endl;
    std::cout << "[Debug] " << (double) static_objs_to_viz_->points.size() / num_pcs_init_ * 100 << setw(7) << "% <- "
              << static_objs_to_viz_->points.size() << " / " << num_pcs_init_ << std::endl;
}

/**
 * @brief 设置路径
 * @param path 输出的导航路径
 * @param mode 路径模式
 * @param node 节点数据
 * @param body2mapprev 车体到地图的变换矩阵
 * @description 将节点的位姿信息添加到导航路径中
 */
void OfflineMapUpdater::set_path(
        nav_msgs::Path &path, std::string mode,
        const erasor::node &node, const Eigen::Matrix4f &body2mapprev) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header          = node.header;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose            = erasor_utils::eigen2geoPose(body2mapprev);

    path.header = pose_stamped.header;
    path.poses.push_back(pose_stamped);
}

/**
 * @brief 发布点云消息（ROS格式）
 * @param map 要发布的点云消息
 * @param publisher ROS发布器
 * @description 发布sensor_msgs::PointCloud2格式的点云消息
 */
void OfflineMapUpdater::publish(
        const sensor_msgs::PointCloud2 &map,
        const ros::Publisher &publisher) {
    pc2_map_.header.frame_id = "map";
    publisher.publish(map);
    if (verbose_) ROS_INFO_STREAM("PC2 is Published!");
}

/**
 * @brief 发布点云消息（PCL格式）
 * @param map 要发布的PCL点云
 * @param publisher ROS发布器
 * @description 将PCL点云转换为ROS消息格式并发布
 */
void OfflineMapUpdater::publish(
        const pcl::PointCloud<pcl::PointXYZI> &map,
        const ros::Publisher &publisher) {
    pcl::toROSMsg(map, pc2_map_);
    pc2_map_.header.frame_id = "map";
    publisher.publish(pc2_map_);
}
