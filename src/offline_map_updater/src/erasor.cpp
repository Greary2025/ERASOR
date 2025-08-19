/**
 * @file erasor.cpp
 * @brief ERASOR算法核心实现文件
 * @author ERASOR团队
 * @description 实现ERASOR（Egocentric Ratio of Pseudo Occupancy-based Dynamic Object Removal）算法
 *              用于从激光雷达点云地图中移除动态对象，保留静态环境结构
 */

#include "erasor/erasor.h"

using namespace std;

/**
 * @brief ERASOR类构造函数
 * @description 初始化ERASOR算法实例
 */
ERASOR::ERASOR() {
}

/**
 * @brief ERASOR类析构函数
 * @description 清理ERASOR算法实例资源
 */
ERASOR::~ERASOR() {
}

/**
 * @brief 将笛卡尔坐标转换为极坐标角度
 * @param x X坐标值
 * @param y Y坐标值
 * @return double 角度值（范围：0 ~ 2π）
 * @description 计算点(x,y)相对于原点的极坐标角度，确保结果在[0, 2π]范围内
 */
double ERASOR::xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
    if (y >= 0) {
        return atan2(y, x); // 第1、2象限
    } else {
        return 2 * PI + atan2(y, x);// 第3、4象限
    }
}

/**
 * @brief 将笛卡尔坐标转换为极坐标半径
 * @param x X坐标值
 * @param y Y坐标值
 * @return double 半径值（距离原点的欧几里得距离）
 * @description 计算点(x,y)到原点的欧几里得距离
 */
double ERASOR::xy2radius(const double &x, const double &y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

/**
 * @brief 清空点云数据
 * @param pt_cloud 待清空的点云引用
 * @description 安全地清空点云数据，避免对空点云进行操作
 */
void ERASOR::clear(pcl::PointCloud<pcl::PointXYZI> &pt_cloud) {
    if (!pt_cloud.empty()) {
        pt_cloud.clear();  ///< 清除点云中的所有点数据
    }
}

/**
 * @brief 初始化径向伪占用描述符（R-POD）数据结构
 * @param r_pod 待初始化的R-POD引用
 * @description 创建极坐标网格结构，用于存储和处理点云数据
 *              R-POD将空间划分为环形（rings）和扇形（sectors）的网格
 */
void ERASOR::init(R_POD &r_pod) {
    if (!r_pod.empty()) {
        r_pod.clear();  ///< 清空现有的R-POD数据
    }
    Ring ring;  ///< 创建环形结构，包含多个扇形Bin
    // 初始化Bin结构：最大高度、最小高度、坐标、占用状态、分配状态
    Bin  bin = {-INF, INF, 0, 0, false, static_cast<bool>(NOT_ASSIGNED)};
    bin.points.reserve(ENOUGH_NUM);  // 预分配足够的内存空间
    
    // 为每个环创建扇形网格
    for (int i = 0; i < num_sectors; i++) {
        ring.emplace_back(bin);  ///< 向环中添加一个扇形Bin
    }
    // 创建完整的径向网格结构
    for (int j = 0; j < num_rings; j++) {
        r_pod.emplace_back(ring);  ///< 向R-POD中添加一个完整的环
    }
}

/**
 * @brief 清空单个Bin的数据
 * @param bin 要清空的Bin引用
 * @description 重置Bin的所有属性到初始状态，包括高度、坐标、占用状态和点云数据
 */
void ERASOR::clear_bin(Bin &bin) {
    bin.max_h       = -INF;        ///< 重置最大高度为负无穷
    bin.min_h       = INF;         ///< 重置最小高度为正无穷
    bin.x           = 0;           ///< 重置X坐标
    bin.y           = 0;           ///< 重置Y坐标
    bin.is_occupied = false;       ///< 重置占用状态为未占用
    bin.status      = NOT_ASSIGNED; ///< 重置分配状态为未分配
    if (!bin.points.empty()) bin.points.clear(); ///< 清空该Bin中的所有点云数据
}

/**
 * @brief 设置输入点云数据（输入应为已变换的点云）
 * @param map_voi 地图感兴趣区域点云
 * @param query_voi 查询感兴趣区域点云
 * @description 初始化所有R-POD结构，清空调试数据，并将输入点云转换为R-POD表示
 *              这是ERASOR算法的主要输入接口
 */
/**
 * @brief 设置ERASOR算法的输入数据
 * @param map_voi 地图感兴趣区域点云
 * @param query_voi 查询感兴趣区域点云（当前帧）
 * @description 清空之前的数据，将输入点云转换为R-POD表示，为后续比较做准备
 */
void ERASOR::set_inputs(
        const pcl::PointCloud<pcl::PointXYZI> &map_voi,
        const pcl::PointCloud<pcl::PointXYZI> &query_voi) {

    // ==================== 清空调试数据 ====================
    clear(debug_curr_rejected);  ///< 清空当前帧被拒绝的点云
    clear(debug_map_rejected);   ///< 清空地图被拒绝的点云
    clear(map_complement);       ///< 清空地图补充点云

    // ==================== 清空R-POD网格数据 ====================
    for (int theta = 0; theta < num_sectors; ++theta) {        ///< 遍历所有扇形
        for (int r = 0; r < num_rings; ++r) {                  ///< 遍历所有环形
            clear_bin(r_pod_map[r][theta]);      ///< 清空地图R-POD的Bin
            clear_bin(r_pod_curr[r][theta]);     ///< 清空当前帧R-POD的Bin
            clear_bin(r_pod_selected[r][theta]); ///< 清空选择结果R-POD的Bin
        }
    }
    
    // ==================== 点云转R-POD表示 ====================
    voi2r_pod(query_voi, r_pod_curr);                          ///< 将查询点云转换为当前帧R-POD
    voi2r_pod(map_voi, r_pod_map, map_complement);             ///< 将地图点云转换为地图R-POD，并分离补充点云

    int      debug_total_num = 0;
    for (int theta           = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            Bin &bin_map = r_pod_map[r][theta];
            debug_total_num += bin_map.points.size();
        }
    }
//  ROS_INFO_STREAM("ERASOR_SRC: \033[1;36m"<<map_voi.points.size()<<"\033[0m =");
//  ROS_INFO_STREAM(" \033[1;36m"<<debug_total_num + map_complement.points.size()<<"\033[0m | "<<debug_total_num << " + \033[1;34m"<<map_complement.points.size()<<"\033[0m");

}

/**
 * @brief 将单个点添加到指定的Bin中
 * @param pt 输入的3D点
 * @param bin 目标Bin引用
 * @description 将点添加到Bin中，并更新Bin的占用状态、高度范围和代表坐标
 *              当点的高度更高时，更新Bin的代表坐标为该点的坐标
 */
void ERASOR::pt2r_pod(const pcl::PointXYZI &pt, Bin &bin) {
    bin.is_occupied = true;        // 标记该Bin为被占用
    bin.points.push_back(pt);      // 将点添加到Bin的点集合中
    
    // 如果当前点的高度大于等于Bin的最大高度，更新最大高度和代表坐标
    if (pt.z >= bin.max_h) {
        bin.max_h = pt.z;          // 更新最大高度
        bin.x     = pt.x;          // 更新代表X坐标
        bin.y     = pt.y;          // 更新代表Y坐标
    }
    // 如果当前点的高度小于等于Bin的最小高度，更新最小高度
    if (pt.z <= bin.min_h) {
        bin.min_h = pt.z;          // 更新最小高度
    }
}

/**
 * @brief 将感兴趣区域点云转换为R-POD表示（单一输出版本）
 * @param src 源点云数据
 * @param r_pod 输出的径向伪占用描述符
 * @description 遍历源点云中的每个点，根据其极坐标位置分配到相应的R-POD网格中
 *              只处理在高度和距离范围内的有效点
 */
void ERASOR::voi2r_pod(
        const pcl::PointCloud<pcl::PointXYZI> &src,
        R_POD &r_pod) {
    // 遍历源点云中的每个点
    for (auto const &pt : src.points) {
        // 检查点的高度是否在有效范围内
        if (pt.z < max_h && pt.z > min_h) {
            double r = xy2radius(pt.x, pt.y);  // 计算点的极坐标半径
            // 检查点的距离是否在有效范围内
            if (r <= max_r) {
                double theta = xy2theta(pt.x, pt.y);  // 计算点的极坐标角度

                // 计算扇形索引（角度索引）
                int sector_idx = min(static_cast<int>((theta / sector_size)), num_sectors - 1);
                // 计算环形索引（径向索引）
                int ring_idx   = min(static_cast<int>((r / ring_size)), num_rings - 1);

                // 将点添加到对应的R-POD网格中
                pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));
            }
        }
    }

    // For debugging
    pcl::PointCloud<pcl::PointXYZI> curr_init;
    r_pod2pc(r_pod, curr_init);
    sensor_msgs::PointCloud2 pc2_curr_init = erasor_utils::cloud2msg(curr_init);
    pub_curr_init.publish(pc2_curr_init);
}

/**
 * @brief 将感兴趣区域点云转换为R-POD表示（带补充点云版本）
 * @param src 源点云数据
 * @param r_pod 输出的径向伪占用描述符
 * @param complement 补充点云，存储超出R-POD范围的点
 * @description 将源点云分为两部分：在R-POD范围内的点进入网格结构，
 *              超出范围的点（距离或高度超限）存储到补充点云中
 */
/**
 * @brief 将感兴趣区域点云转换为R-POD表示（双输出版本）
 * @param src 源点云数据
 * @param r_pod 输出的径向伪占用描述符
 * @param complement 输出的补充点云（超出范围的点）
 * @description 遍历源点云中的每个点，根据其极坐标位置分配到相应的R-POD网格中，
 *              将超出高度或距离范围的点分离到补充点云中
 */
void ERASOR::voi2r_pod(
        const pcl::PointCloud<pcl::PointXYZI> &src,
        R_POD &r_pod, pcl::PointCloud<pcl::PointXYZI> &complement) {
    // ==================== 遍历处理每个点 ====================
    for (auto const &pt : src.points) {
        // ==================== 高度范围检查 ====================
        if (pt.z < max_h && pt.z > min_h) {  ///< 检查点的高度是否在有效范围内
            double r = xy2radius(pt.x, pt.y);  ///< 计算点的极坐标半径
            
            // ==================== 距离范围检查 ====================
            if (r <= max_r) {  ///< 检查点的距离是否在有效范围内
                double theta      = xy2theta(pt.x, pt.y);  ///< 计算点的极坐标角度
                
                // ==================== 计算网格索引 ====================
                int    sector_idx = min(static_cast<int>((theta / sector_size)), num_sectors - 1);  ///< 计算扇形索引（角度索引）
                int    ring_idx   = min(static_cast<int>((r / ring_size)), num_rings - 1);          ///< 计算环形索引（径向索引）
                
                // ==================== 添加到R-POD网格 ====================
                pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));  ///< 将点添加到对应的R-POD网格中
            } else { 
            // ==================== 距离超出范围处理 ====================
            complement.points.push_back(pt);  ///< 距离超出范围的点添加到补充点云
        }
    } else { 
        // ==================== 高度超出范围处理 ====================
        complement.points.push_back(pt);  ///< 高度超出范围的点添加到补充点云
    }
    }
    
    // ==================== 可视化发布 ====================
    pcl::PointCloud<pcl::PointXYZI> map_init;                                ///< 临时点云用于可视化
    r_pod2pc(r_pod, map_init);                                               ///< 将R-POD转换回点云格式
    sensor_msgs::PointCloud2 pc2_map_init = erasor_utils::cloud2msg(map_init); ///< 转换为ROS消息格式
    pub_map_init.publish(pc2_map_init);                                      ///< 发布初始地图点云用于可视化
}

/**
 * @brief 可视化伪占用信息
 * @description 创建多边形数组来可视化地图和当前帧的R-POD结构
 *              使用伪占用度（高度差）作为可视化的强度值
 */
void ERASOR::viz_pseudo_occupancy() {
    jsk_recognition_msgs::PolygonArray map_r_pod, curr_r_pod;
    // 设置地图R-POD可视化的消息头
    map_r_pod.header.frame_id = "map";
    map_r_pod.header.stamp    = ros::Time::now();

    // 设置当前帧R-POD可视化的消息头
    curr_r_pod.header.frame_id = "map";
    curr_r_pod.header.stamp    = ros::Time::now();

    // 遍历所有R-POD网格
    for (int theta = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            Bin  &bin_curr = r_pod_curr[r][theta];   // 当前帧的Bin
            Bin  &bin_map  = r_pod_map[r][theta];    // 地图的Bin
            auto polygons  = set_polygons(r, theta, 3); // 创建多边形表示
            polygons.header = curr_r_pod.header;
            map_r_pod.polygons.push_back(polygons);
            curr_r_pod.polygons.push_back(polygons);

            // 计算当前帧的伪占用度（归一化的高度差）
            if (bin_curr.is_occupied) {
                double curr_pod = bin_curr.max_h - bin_curr.min_h;
                curr_r_pod.likelihood.push_back(curr_pod / (max_h - min_h));
            } else {
                curr_r_pod.likelihood.push_back(LITTLE_NUM); // 未占用时使用极小值
            }
            // 计算地图的伪占用度（归一化的高度差）
            if (bin_map.is_occupied) {
                double map_pod = bin_map.max_h - bin_map.min_h;
                map_r_pod.likelihood.push_back(map_pod / (max_h - min_h));
            } else {
                map_r_pod.likelihood.push_back(LITTLE_NUM); // 未占用时使用极小值
            }
        }
    }

    // 发布可视化消息
    pub_map_marker.publish(map_r_pod);
    pub_curr_marker.publish(curr_r_pod);
}


/**
 * @brief 估计地面平面参数
 * @param ground 地面种子点云
 * @description 使用主成分分析（PCA）和奇异值分解（SVD）来估计地面平面的法向量和距离参数
 *              地面平面方程：normal^T * [x,y,z] + d = 0
 */
void ERASOR::estimate_plane_(const pcl::PointCloud<pcl::PointXYZI> &ground) {
    Eigen::Matrix3f cov;        // 协方差矩阵
    Eigen::Vector4f pc_mean;    // 点云均值
    // 计算点云的均值和协方差矩阵
    pcl::computeMeanAndCovarianceMatrix(ground, cov, pc_mean);
    
    // 奇异值分解（SVD）
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    // 使用最小奇异值对应的向量作为法向量（地面法向量）
    normal_ = (svd.matrixU().col(2));
    // 地面种子点的均值
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // 根据平面方程 normal^T * [x,y,z] = -d 计算距离参数
    d_         = -(normal_.transpose() * seeds_mean)(0, 0);
    // 设置距离阈值为 th_dist - d
    th_dist_d_ = th_dist_ - d_;
}

/**
 * @brief 点云高度比较函数
 * @param a 第一个点
 * @param b 第二个点
 * @return bool 如果点a的高度小于点b的高度则返回true
 * @description 用于按Z坐标（高度）对点云进行排序的比较函数
 */
bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b) {
    return a.z < b.z;
}

/**
 * @brief 提取地面初始种子点
 * @param p_sorted 按高度排序的点云
 * @param init_seeds 输出的初始种子点云
 * @description 使用LPR（Low Point Representative）方法提取地面种子点
 *              首先计算最低点的平均高度，然后选择高度接近该平均值的点作为种子点
 */
void ERASOR::extract_initial_seeds_(
        const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
        pcl::PointCloud<pcl::PointXYZI> &init_seeds) {
    init_seeds.points.clear();
    pcl::PointCloud<pcl::PointXYZI> g_seeds_pc;

    // LPR是低点代表的平均值
    double sum = 0;
    int    cnt = 0;

    // 计算最低点的平均高度值
    for (int i = num_lowest_pts; i < p_sorted.points.size() && cnt < num_lprs_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // 防止除零错误
    g_seeds_pc.clear();
    
    // 遍历点云，筛选高度小于 lpr_height + th_seeds_heights_ 的点
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_heights_) {
            g_seeds_pc.points.push_back(p_sorted.points[i]);
        }
    }
    // 返回种子点
    init_seeds = g_seeds_pc;
}

/**
 * @brief 提取地面点云
 * @param src 源点云数据
 * @param dst 输出的地面点云
 * @param outliers 输出的非地面点云（障碍物）
 * @description 使用迭代地面滤波算法提取地面点云
 *              1. 移除异常值（过低的点）
 *              2. 提取初始种子点
 *              3. 迭代优化地面平面模型
 */
void ERASOR::extract_ground(
        const pcl::PointCloud<pcl::PointXYZI> &src,
        pcl::PointCloud<pcl::PointXYZI> &dst, pcl::PointCloud<pcl::PointXYZI> &outliers) {
    if (!dst.empty()) dst.clear();
    if (!outliers.empty()) outliers.clear();

    auto src_copy = src;
    // 按高度对点云进行排序
    std::sort(src_copy.points.begin(), src_copy.points.end(), point_cmp);
    
    // 1. 移除异常值（过低的点）
    auto     it = src_copy.points.begin();
    for (int i  = 0; i < src_copy.points.size(); i++) {
        if (src_copy.points[i].z < min_h) {
            it++;
        } else {
            break;
        }
    }
    src_copy.points.erase(src_copy.points.begin(), it);

    // 2. 设置种子点
    if (!ground_pc_.empty()) ground_pc_.clear();
    if (!non_ground_pc_.empty()) non_ground_pc_.clear();

    extract_initial_seeds_(src_copy, ground_pc_);
    
    // 3. 迭代提取地面
    for (int i = 0; i < iter_groundfilter_; i++) {
        estimate_plane_(ground_pc_);  // 估计地面平面
        ground_pc_.clear();

        // 将点云转换为矩阵形式
        Eigen::MatrixXf points(src.points.size(), 3);
        int             j      = 0;
        for (auto       p:src.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // 应用地面平面模型
        Eigen::VectorXf result = points * normal_;
        // 阈值滤波
        for (int        r      = 0; r < result.rows(); r++) {
            if (result[r] < th_dist_d_) {
                ground_pc_.points.push_back(src[r]);  // 地面点
            } else {
                if (i == (iter_groundfilter_ - 1)) { // 最后一次迭代
                    non_ground_pc_.points.push_back(src[r]);  // 非地面点
                }
            }
        }
    }
    dst      = ground_pc_;      // 输出地面点云
    outliers = non_ground_pc_;  // 输出非地面点云
}

/**
 * @brief 合并两个Bin的数据
 * @param src1 第一个源Bin
 * @param src2 第二个源Bin
 * @param dst 输出的合并后Bin
 * @description 将两个Bin的点云数据合并，取高度范围的并集
 *              用于处理扫描比率正常的情况，保留两帧的所有信息
 */
void ERASOR::merge_bins(const Bin &src1, const Bin &src2, Bin &dst) {
    dst.max_h       = max(src1.max_h, src2.max_h);  ///< 取两个Bin的最大高度
    dst.min_h       = min(src1.min_h, src2.min_h);  ///< 取两个Bin的最小高度
    dst.is_occupied = true;                         ///< 标记合并后的Bin为占用状态
    dst.points.clear();  ///< 清空目标Bin的点云数据
    
    // ==================== 合并点云数据 ====================
    for (auto const &pt : src1.points) {  ///< 添加第一个Bin的所有点
        dst.points.push_back(pt);
    }
    for (auto const &pt : src2.points) {  ///< 添加第二个Bin的所有点
        dst.points.push_back(pt);
    }
}

/**
 * @brief 将R-POD结构转换为点云
 * @param sc 输入的R-POD结构
 * @param pc 输出的点云
 * @description 遍历R-POD中所有被占用的网格，将其中的点云数据提取出来
 */
void ERASOR::r_pod2pc(const R_POD &sc, pcl::PointCloud<pcl::PointXYZI> &pc) {
    pc.points.clear();
    // 遍历所有扇形和环形网格
    for (int theta = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            // 如果网格被占用，提取其中的所有点
            if (sc.at(r).at(theta).is_occupied) {
                for (auto const &pt : sc.at(r).at(theta).points) {
                    pc.points.push_back(pt);
                }
            }
        }
    }
}

/**
 * @brief 获取被拒绝的异常点云
 * @param map_rejected 输出的地图中被拒绝的点云
 * @param curr_rejected 输出的当前帧中被拒绝的点云
 * @description 返回在动态对象检测过程中被标记为异常的点云数据，用于调试和分析
 */
void ERASOR::get_outliers(
        pcl::PointCloud<pcl::PointXYZI> &map_rejected,
        pcl::PointCloud<pcl::PointXYZI> &curr_rejected) {
    map_rejected  = debug_map_rejected;   // 地图中的异常点
    curr_rejected = debug_curr_rejected;  // 当前帧中的异常点
}

/**
 * @brief 比较地图和当前帧的R-POD并恢复地面点
 * @description ERASOR算法的核心比较功能，通过扫描比率测试检测动态对象
 *              并使用R-GPF（径向地面平面拟合）恢复被遮挡的地面点
 * @param frame 当前帧编号，用于调试和可视化
 */
void ERASOR::compare_vois_and_revert_ground(int frame) {
    // ==================== 可视化初始化 ====================
    jsk_recognition_msgs::PolygonArray poly_list;  ///< 多边形数组用于可视化不同类型的Bin
    poly_list.header.frame_id = "map";
    poly_list.header.stamp    = ros::Time::now();

    int        dynamic_count;  ///< 动态对象计数器
    static int cnt            = 0;  ///< 静态帧计数器
    ground_viz.points.clear();  ///< 清空地面可视化点云

//  std::string filename = "/home/shapelim/debug/" + std::to_string(frame) +".csv";
//  ofstream output(filename.data());
//  output<<"which_one_is_higher,r,theta,min_h,max_h,diff_h,min_h,max_h,diff_h,ratio\n";

    cnt++;  ///< 增加帧计数器
    
    // ==================== 双重循环遍历R-POD网格 ====================
    for (int theta = 0; theta < num_sectors; theta++) {  ///< 遍历所有扇形sector
        for (int r = 0; r < num_rings; r++) {  ///< 遍历所有环形ring

            // ==================== 获取当前Bin引用 ====================
            Bin &bin_curr = r_pod_curr[r][theta];  ///< 当前帧的Bin
            Bin &bin_map  = r_pod_map[r][theta];   ///< 地图的Bin

            // ==================== 最小点数过滤 ====================
            if (bin_curr.points.size() < minimum_num_pts) {
                r_pod_selected[r][theta] = bin_map;  ///< 点数不足时直接使用地图Bin
//        debug_curr_rejected += bin_curr.points;  ///< 调试：记录被拒绝的点

                auto polygons = set_polygons(r, theta, 3);  ///< 创建可视化多边形
                polygons.header = poly_list.header;
                poly_list.polygons.push_back(polygons);
                poly_list.likelihood.push_back(LITTLE_NUM);  ///< 标记为点数不足类型

                continue;  ///< 跳过后续处理

            }
            // ==================== 双占用Bin处理 ====================
            if (bin_curr.is_occupied && bin_map.is_occupied) {
                // ==================== 扫描比率测试 ====================
                // 通过比较高度差来检测动态对象
                double map_h_diff  = bin_map.max_h - bin_map.min_h;    ///< 地图Bin的高度差
                double curr_h_diff = bin_curr.max_h - bin_curr.min_h;  ///< 当前帧Bin的高度差
                double scan_ratio  = min(map_h_diff / curr_h_diff,     ///< 计算扫描比率（取较小值）
                                         curr_h_diff / map_h_diff);

                if (scan_ratio < scan_ratio_threshold) { ///< 扫描比率低于阈值，检测到动态对象

                    if (map_h_diff >= curr_h_diff) { ///< 情况1：对象消失（地图高度差更大）
                        auto polygons = set_polygons(r, theta, 3);  ///< 创建绿色可视化多边形
                        polygons.header = poly_list.header;
                        poly_list.polygons.push_back(polygons);
                        poly_list.likelihood.push_back(MAP_IS_HIGHER);  ///< 标记为地图更高类型

                        if (bin_map.max_h > th_bin_max_h) {  ///< 地图Bin高度超过阈值
                            r_pod_selected[r][theta] = bin_curr;  ///< 选择当前帧Bin作为基础
                            // ==================== R-GPF地面恢复操作 ====================
                            // 从地图Bin中提取地面点，恢复被动态对象遮挡的地面
                            if (!piecewise_ground_.empty()) piecewise_ground_.clear();  ///< 清空地面点云
                            if (!non_ground_.empty()) non_ground_.clear();              ///< 清空非地面点云
                            extract_ground(bin_map.points, piecewise_ground_, non_ground_);  ///< 地面提取
                            r_pod_selected[r][theta].points += piecewise_ground_;  ///< 添加恢复的地面点
                            ground_viz += piecewise_ground_;     ///< 添加到地面可视化
                            debug_map_rejected += non_ground_;   ///< 记录被拒绝的非地面点
                        } else {
                            r_pod_selected[r][theta] = bin_map;  ///< 高度不足时直接使用地图Bin
                        }
                    } else if (map_h_diff <= curr_h_diff) { ///< 情况2：对象出现（当前帧高度差更大）
                        auto polygons = set_polygons(r, theta, 3);  ///< 创建红色可视化多边形
                        polygons.header = poly_list.header;
                        poly_list.polygons.push_back(polygons);
                        poly_list.likelihood.push_back(CURR_IS_HIGHER);  ///< 标记为当前帧更高类型

                        r_pod_selected[r][theta] = bin_map;  ///< 保持地图Bin（拒绝新出现的对象）
                        if (bin_curr.max_h > th_bin_max_h) {  ///< 当前帧高度超过阈值
                            //            r_pod_selected[r][theta] = bin_curr;  ///< 可选：使用当前帧Bin
                            debug_curr_rejected += bin_curr.points;  ///< 记录被拒绝的当前帧点
                        }
                    }

                } else {  ///< 扫描比率正常，无动态对象检测
                    // ==================== Bin合并处理 ====================
                    auto polygons = set_polygons(r, theta, 3);  ///< 创建蓝色可视化多边形
                    polygons.header = poly_list.header;
                    poly_list.polygons.push_back(polygons);
                    poly_list.likelihood.push_back(MERGE_BINS);  ///< 标记为合并Bin类型
                    Bin bin_merged;  ///< 创建合并后的Bin
                    merge_bins(bin_curr, bin_map, bin_merged);  ///< 合并两个Bin
                    r_pod_selected[r][theta] = bin_merged;  ///< 使用合并后的Bin
                }
            // ==================== 单占用Bin处理 ====================
            } else if (bin_curr.is_occupied) {  ///< 仅当前帧Bin有占用
                r_pod_selected[r][theta] = bin_curr;  ///< 直接使用当前帧Bin
            } else if (bin_map.is_occupied) {    ///< 仅地图Bin有占用
                r_pod_selected[r][theta] = bin_map;   ///< 直接使用地图Bin
            }
            // 注意：两个Bin都未占用时，r_pod_selected保持默认状态（未占用）

        }
    }
    // For debugging
    sensor_msgs::PointCloud2 pc2_map_r  = erasor_utils::cloud2msg(debug_map_rejected);
    sensor_msgs::PointCloud2 pc2_curr_r = erasor_utils::cloud2msg(debug_curr_rejected);
    pub_map_rejected.publish(pc2_map_r);
    pub_curr_rejected.publish(pc2_curr_r);
    pub_viz_bin_marker.publish(poly_list);
}

// Version 3.
// Retrieve piecewise with blocking!
void ERASOR::compare_vois_and_revert_ground_w_block(int frame) {
    jsk_recognition_msgs::PolygonArray poly_list;
    poly_list.header.frame_id = "map";
    poly_list.header.stamp    = ros::Time::now();

    int dynamic_count;

    ground_viz.points.clear();

    // 1. Update status!!
    for (int theta = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            // Min. num of pts criteria.
            Bin &bin_curr = r_pod_curr[r][theta];
            Bin &bin_map  = r_pod_map[r][theta];

            if (bin_map.points.empty()) {
                r_pod_selected[r][theta].status = LITTLE_NUM;
                continue;
            }

            if (bin_curr.points.size() < minimum_num_pts) {
                r_pod_selected[r][theta].status = LITTLE_NUM;
            } else {
                double map_h_diff  = bin_map.max_h - bin_map.min_h;
                double curr_h_diff = bin_curr.max_h - bin_curr.min_h;
                double scan_ratio  = min(map_h_diff / curr_h_diff,
                                         curr_h_diff / map_h_diff);
                // ---------------------------------
                //          Scan Ratio Test
                // ---------------------------------
                if (bin_curr.is_occupied && bin_map.is_occupied) {
                    if (scan_ratio < scan_ratio_threshold) { // find dynamic!
                        if (map_h_diff >= curr_h_diff) { // Occupied -> Disappear  <<BLUE>>
                            r_pod_selected[r][theta].status = MAP_IS_HIGHER;
                        } else if (map_h_diff <= curr_h_diff) { // No objects exist -> Appear! <<GREEN>>
                            r_pod_selected[r][theta].status = CURR_IS_HIGHER;
                        }
                    } else {
                        r_pod_selected[r][theta].status = MERGE_BINS;
                    }
                } else if (bin_map.is_occupied) { // Maybe redundant?
                    r_pod_selected[r][theta].status = LITTLE_NUM;
                }
            }

        }
//    out_map<<"\n";  out_curr<<"\n";
    }

//  pcl::PointCloud<pcl::PointXYZI> origin_total, ground_total, dummy_non_ground;
    int num_origin_stat, num_origin_dyn;
    int num_ground_stat, num_ground_dyn;

    // 2. set bins!
    for (int theta = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            //visualization
            auto polygons = set_polygons(r, theta, 3);
            polygons.header = poly_list.header;
            poly_list.polygons.push_back(polygons);

            Bin &bin_curr = r_pod_curr[r][theta];
            Bin &bin_map  = r_pod_map[r][theta];

            double OCCUPANCY_STATUS = r_pod_selected[r][theta].status;
            if (OCCUPANCY_STATUS == LITTLE_NUM) {
                r_pod_selected[r][theta] = bin_map;
                r_pod_selected[r][theta].status = LITTLE_NUM;

                poly_list.likelihood.push_back(LITTLE_NUM);

            } else if (OCCUPANCY_STATUS == MAP_IS_HIGHER) {
                if ((bin_map.max_h - bin_map.min_h) > 0.5) {
                    r_pod_selected[r][theta] = bin_curr;
                    r_pod_selected[r][theta].status = MAP_IS_HIGHER;
                    // ---------------------------------
                    //     NOTE: Ground is retrieved!
                    // ---------------------------------

                    if (!piecewise_ground_.empty()) piecewise_ground_.clear();
                    if (!non_ground_.empty()) non_ground_.clear();

                    extract_ground(bin_map.points, piecewise_ground_, non_ground_);
                    /*** It potentially requires lots of memories... */
                    r_pod_selected[r][theta].points += piecewise_ground_;

                    /*** Thus, voxelization is conducted */
                    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
                    *tmp = r_pod_selected[r][theta].points;
                    erasor_utils::voxelize_preserving_labels(tmp, r_pod_selected[r][theta].points, map_voxel_size_);

                    erasor_utils::count_stat_dyn(piecewise_ground_, num_ground_stat, num_ground_dyn);
                    ground_viz += piecewise_ground_;
                    debug_map_rejected += non_ground_;

                    poly_list.likelihood.push_back(MAP_IS_HIGHER);
                } else {
                    r_pod_selected[r][theta] = bin_map;
                    r_pod_selected[r][theta].status = NOT_ASSIGNED;

                    poly_list.likelihood.push_back(NOT_ASSIGNED);
                }

            } else if (OCCUPANCY_STATUS == CURR_IS_HIGHER) {
                r_pod_selected[r][theta] = bin_map;
                r_pod_selected[r][theta].status = CURR_IS_HIGHER;

                poly_list.likelihood.push_back(CURR_IS_HIGHER);

            } else if (OCCUPANCY_STATUS == MERGE_BINS) {
                if (is_dynamic_obj_close(r_pod_selected, r, theta, 1, 1)) {
                    r_pod_selected[r][theta] = bin_map;
                    r_pod_selected[r][theta].status = BLOCKED;

                    poly_list.likelihood.push_back(BLOCKED);
                } else {
                    // NOTE the dynamic object comes ....:(
                    r_pod_selected[r][theta] = bin_map;
                    r_pod_selected[r][theta].status = MERGE_BINS;

                    poly_list.likelihood.push_back(MERGE_BINS);
                }
            }
        }
    }

    // For debugging
    sensor_msgs::PointCloud2 pc2_map_r  = erasor_utils::cloud2msg(debug_map_rejected);
    sensor_msgs::PointCloud2 pc2_curr_r = erasor_utils::cloud2msg(debug_curr_rejected);
    pub_map_rejected.publish(pc2_map_r);
    pub_curr_rejected.publish(pc2_curr_r);
    pub_viz_bin_marker.publish(poly_list);
}

/**
 * @brief 检测目标位置附近是否有动态对象
 * @param r_pod_selected 选定的R-POD结构
 * @param r_target 目标环形索引
 * @param theta_target 目标扇形索引
 * @param r_range 环形搜索范围
 * @param theta_range 扇形搜索范围
 * @return true 如果附近有动态对象，false 否则
 * @description 在指定位置周围的邻域内搜索是否存在标记为CURR_IS_HIGHER的Bin
 *              用于判断当前位置是否受到动态对象的影响
 */
bool ERASOR::is_dynamic_obj_close(R_POD &r_pod_selected, int r_target, int theta_target, int r_range, int theta_range) {
    // ==================== 构建扇形候选索引 ====================
    std::vector<int> theta_candidates;  ///< 扇形候选索引列表
    for (int         j = theta_target - theta_range; j <= theta_target + theta_range; j++) {
        if (j < 0) {  ///< 处理负索引的环形边界
            theta_candidates.push_back(j + num_rings);
        } else if (j >= num_sectors) {  ///< 处理超出范围的环形边界
            theta_candidates.push_back(j - num_rings);
        } else {
            theta_candidates.push_back(j);  ///< 正常范围内的索引
        }
    }
    
    // ==================== 搜索邻域内的动态对象 ====================
    for (int         r = std::max(0, r_target - r_range); r <= std::min(r_target + r_range, num_rings - 1); r++) {
        for (const auto &theta:theta_candidates) {
            if ((r == r_target) && (theta == theta_target)) continue;  ///< 跳过目标位置本身

            if (r_pod_selected[r][theta].status == CURR_IS_HIGHER) {  ///< 检测到动态对象
                return true;
            }
        }
    }
    return false;  ///< 未检测到动态对象
}

/**
 * @brief 检测Bin中是否包含动态对象
 * @param bin 待检测的Bin
 * @return true 如果包含动态对象，false 否则
 * @description 通过解析点云强度值中的语义标签，判断Bin中是否包含
 *              属于动态类别（如车辆、行人等）的点云数据
 */
bool ERASOR::has_dynamic(Bin &bin) {
    for (const auto &pt:bin.points) {  ///< 遍历Bin中的所有点
        uint32_t float2int      = static_cast<uint32_t>(pt.intensity);  ///< 将强度值转换为整数
        uint32_t semantic_label = float2int & 0xFFFF;  ///< 提取语义标签（低16位）

        for (int class_num: DYNAMIC_CLASSES) {  ///< 遍历所有动态类别
            if (semantic_label == class_num) {  ///< 检查是否属于动态对象类别
                return true;
            }
        }
    }
    return false;  ///< 未发现动态对象
}

/**
 * @brief 获取静态环境估计结果
 * @param arranged 输出的整理后点云（包含静态结构）
 * @param complement 输出的补充点云
 * @description 将R-POD结构转换为最终的静态点云，并添加恢复的地面点
 *              这是ERASOR算法的最终输出，代表去除动态对象后的静态环境
 */
void ERASOR::get_static_estimate(
        pcl::PointCloud<pcl::PointXYZI> &arranged,
        pcl::PointCloud<pcl::PointXYZI> &complement) {
    r_pod2pc(r_pod_selected, arranged);  ///< 将选定的R-POD转换为点云
    arranged += ground_viz;  ///< 添加恢复的地面点云
    if(ground_viz.size() != 0){  ///< 如果有地面可视化数据
        sensor_msgs::PointCloud2 pc2_ground = erasor_utils::cloud2msg(ground_viz);  ///< 转换为ROS消息
        pub_ground.publish(pc2_ground);
    }

    complement = map_complement;
    sensor_msgs::PointCloud2 pc2_arranged       = erasor_utils::cloud2msg(arranged);
    sensor_msgs::PointCloud2 pc2_map_complement = erasor_utils::cloud2msg(complement);
    pub_arranged.publish(pc2_arranged);
}

double ERASOR::get_max_range() {return max_r;}

geometry_msgs::PolygonStamped ERASOR::set_polygons(int r_idx, int theta_idx, int num_split) {
    geometry_msgs::PolygonStamped polygons;
    // Set point of polygon. Start from RL and ccw
    geometry_msgs::Point32        point;

    point.z = max_h + 0.5;
    // RL
    double r_len = r_idx * ring_size;
    double angle = theta_idx * sector_size;

    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    polygons.polygon.points.push_back(point);
    // RU
    r_len = r_len + ring_size;
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    polygons.polygon.points.push_back(point);

    // RU -> LU
    for (int idx = 1; idx <= num_split; ++idx) {
        angle = angle + sector_size / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        polygons.polygon.points.push_back(point);
    }

    r_len = r_len - ring_size;
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    polygons.polygon.points.push_back(point);

    for (int idx = 1; idx < num_split; ++idx) {
        angle = angle - sector_size / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        polygons.polygon.points.push_back(point);
    }

    return polygons;
}


