/**
 * @file erasor.h
 * @brief ERASOR (Egocentric Ratio of Pseudo Occupancy-based Dynamic Object Removal) 核心头文件
 * @author ERASOR团队
 * @description 该文件定义了ERASOR算法的核心类和数据结构，用于动态对象检测和移除
 */

#include "tools/erasor_utils.hpp"

// ==================== 常量定义 ====================

/** @brief 无穷大值，用于初始化 */
#define INF 10000000000000.0

/** @brief 圆周率 */
#define PI 3.1415926535

/** @brief 每个bin中预分配的点数，用于性能优化 */
#define ENOUGH_NUM 8000

// ==================== 状态定义 ====================

/** @brief 空状态 */
#define EMPTY 0

/** @brief 地图状态 */
#define MAP 1

/** @brief 当前点云状态 */
#define PC_CURR 2

// ==================== 可视化颜色定义 ====================
// 用于RViz中的颜色编码显示
// 0 -> 蓝色

/** @brief 地图高度更高时的颜色值 */
#define MAP_IS_HIGHER 0.5

/** @brief 当前点云高度更高时的颜色值 */
#define CURR_IS_HIGHER 1.0

/** @brief 未激活状态的颜色值（蓝色） */
#define LITTLE_NUM 0.0

/** @brief 被阻挡状态的颜色值 */
#define BLOCKED 0.8

// ==================== 合并和分配定义 ====================

/** @brief bin合并的阈值 */
#define MERGE_BINS 0.25

/** @brief 未分配状态 */
#define NOT_ASSIGNED 0.0

using namespace std;

/**
 * @struct Bin
 * @brief 极坐标网格中的基本单元
 * @description 在R-POD（Range-Polar Occupancy Descriptor）结构中，每个bin存储一个扇形区域内的点云信息
 */
struct Bin {
    double max_h;        ///< 该bin中点的最大高度
    double min_h;        ///< 该bin中点的最小高度
    double x;            ///< 最高点的x坐标
    double y;            ///< 最高点的y坐标
    double status;       ///< bin的状态标识
    bool   is_occupied;  ///< 是否被占用

    pcl::PointCloud<pcl::PointXYZI> points;  ///< 该bin中包含的所有点
};

/**
 * @struct DynamicBinIdx
 * @brief 动态bin的索引结构
 * @description 用于标识R-POD结构中动态对象所在的bin位置
 */
struct DynamicBinIdx {
    int r;      ///< 径向索引（距离环）
    int theta;  ///< 角度索引（扇形）
};

/**
 * @typedef R_POD
 * @brief Range-Polar Occupancy Descriptor 数据结构
 * @description 二维向量，表示极坐标系下的占用网格
 */
typedef vector<vector<Bin> > R_POD;

/**
 * @typedef Ring
 * @brief 距离环数据结构
 * @description 表示某个距离环上的所有bin
 */
typedef vector<Bin> Ring;

/**
 * @class ERASOR
 * @brief ERASOR算法的核心类
 * @description 实现基于自中心伪占用比率的动态对象移除算法
 */
class ERASOR {
public:

    /**
     * @brief ERASOR构造函数
     * @param nodehandler ROS节点句柄指针
     * @description 初始化ERASOR算法的所有参数、发布器和数据结构
     */
    ERASOR(ros::NodeHandle *nodehandler) : nh(*nodehandler) {
        // ==================== 参数加载 ====================
        
        // 基本几何参数
        nh.param("/erasor/max_range", max_r, 10.0);                    ///< 最大检测距离（米）
        nh.param("/erasor/num_rings", num_rings, 20);                  ///< 径向环数
        nh.param("/erasor/num_sectors", num_sectors, 60);              ///< 角度扇区数
        nh.param("/erasor/max_h", max_h, 3.0);                         ///< 最大高度阈值（米）
        nh.param("/erasor/min_h", min_h, 0.0);                         ///< 最小高度阈值（米）
        
        // 动态对象检测参数
        nh.param("/erasor/th_bin_max_h", th_bin_max_h, 0.39);          ///< bin最大高度阈值
        nh.param("/erasor/scan_ratio_threshold", scan_ratio_threshold, 0.22);  ///< 扫描比率阈值
        nh.param("/erasor/num_lowest_pts", num_lowest_pts, 5);         ///< 最低点数量
        nh.param("/erasor/minimum_num_pts", minimum_num_pts, 4);       ///< 最小点数阈值
        nh.param("/erasor/rejection_ratio", rejection_ratio, 0.33);    ///< 拒绝比率
        
        // 地面滤波参数
        nh.param("/erasor/gf_dist_thr", th_dist_, 0.05);               ///< 地面滤波距离阈值
        nh.param("/erasor/gf_iter", iter_groundfilter_, 3);            ///< 地面滤波迭代次数
        nh.param("/erasor/gf_num_lpr", num_lprs_, 10);                 ///< 最低点区域数量
        nh.param("/erasor/gf_th_seeds_height", th_seeds_heights_, 0.5); ///< 种子点高度阈值
        
        // 地图处理参数
        nh.param("/erasor/map_voxel_size", map_voxel_size_, 0.2);      ///< 地图体素大小

        // ==================== 计算派生参数 ====================
        ring_size   = max_r / num_rings;        ///< 每个环的径向大小
        sector_size = 2 * PI / num_sectors;     ///< 每个扇区的角度大小

        // ==================== 初始化发布器 ====================
        // SCDR是项目的原始名称
        
        // 调试用点云发布器
        pub_map_rejected  = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/map_rejected", 100);   ///< 被拒绝的地图点
        pub_curr_rejected = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/curr_rejected", 100);  ///< 被拒绝的当前点
        pub_map_init      = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/map_init", 100);       ///< 初始地图
        pub_curr_init     = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/curr_init", 100);      ///< 初始当前点云

        // 可视化标记发布器
        pub_map_marker     = nh.advertise<jsk_recognition_msgs::PolygonArray>("/SCDR/debug/map_marker", 100);     ///< 地图标记
        pub_curr_marker    = nh.advertise<jsk_recognition_msgs::PolygonArray>("/SCDR/debug/curr_marker", 100);    ///< 当前标记
        pub_viz_bin_marker = nh.advertise<jsk_recognition_msgs::PolygonArray>("/SCDR/debug/polygons_marker", 100); ///< bin可视化标记

        // 地面和排列点云发布器
        pub_ground   = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/ground", 100);     ///< 地面点云
        pub_arranged = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/arranged", 100);   ///< 排列后的点云

        // ==================== 打印参数信息 ====================
        std::cout << "-----\033[1;32mParams. of ERASOR\033[0m-----" << std::endl;
        std::cout << "max range: " << max_r << std::endl;
        std::cout << "num_rings: " << num_rings << std::endl;
        std::cout << "num_sectors: " << num_sectors << std::endl;
        std::cout << "min_h: " << min_h << std::endl;
        std::cout << "max_h: " << max_h << std::endl;
        std::cout << "scan_ratio_threshold: " << scan_ratio_threshold << std::endl;
        std::cout << "th_bin_max_h: " << th_bin_max_h << std::endl;
        std::cout << "minimum_num_pts: " << minimum_num_pts << std::endl;
        std::cout << "rejection_ratio: " << rejection_ratio << std::endl;
        std::cout << th_dist_ << std::endl;
        std::cout << iter_groundfilter_ << std::endl;
        std::cout << num_lprs_ << std::endl;
        std::cout << th_seeds_heights_ << std::endl;
        std::cout << "-----------------------" << std::endl;
        
        // ==================== 初始化R-POD数据结构 ====================
        init(r_pod_map);        ///< 初始化地图R-POD
        init(r_pod_curr);       ///< 初始化当前点云R-POD
        init(r_pod_selected);   ///< 初始化选择的R-POD

        // ==================== 预分配内存 ====================
        // 为性能优化预分配足够的内存空间
        piecewise_ground_.reserve(130000);   ///< 分段地面点云
        non_ground_.reserve(130000);         ///< 非地面点云
        ground_pc_.reserve(130000);          ///< 地面点云
        non_ground_pc_.reserve(130000);      ///< 非地面点云
    }
    
    /**
     * @brief 默认构造函数
     */
    ERASOR();

    /**
     * @brief 析构函数
     */
    ~ERASOR();

    // ==================== 核心接口方法 ====================
    
    /**
     * @brief 设置输入点云数据
     * @param map_voi 地图感兴趣区域点云（已变换和裁剪）
     * @param query_voi 查询感兴趣区域点云（已变换和裁剪）
     * @description 将输入的地图和查询点云转换为R-POD格式，为后续处理做准备
     */
    void set_inputs(
            const pcl::PointCloud<pcl::PointXYZI> &map_voi,
            const pcl::PointCloud<pcl::PointXYZI> &query_voi);

    /**
     * @brief 比较并选择静态点云
     * @param frame 当前帧号
     * @description 比较地图和当前点云，选择静态部分
     */
    void compare_then_select(int frame);

    /**
     * @brief 获取处理后的点云
     * @param arranged 输出的排列点云
     * @param complement 输出的补充点云
     * @description 从R-POD结构中提取处理后的点云数据
     */
    void get_pcs(
            pcl::PointCloud<pcl::PointXYZI> &arranged,
            pcl::PointCloud<pcl::PointXYZI> &complement);

    // ==================== 版本2算法 ====================
    
    /**
     * @brief 比较VoI并恢复地面点（版本2）
     * @param frame 当前帧号
     * @description 实现ERASOR算法的第二版本，包含地面点恢复功能
     */
    void compare_vois_and_revert_ground(int frame);

    /**
     * @brief 获取静态估计结果
     * @param arranged 输出的静态点云
     * @param complement 输出的补充点云
     * @description 提取算法估计的静态环境点云
     */
    void get_static_estimate(
            pcl::PointCloud<pcl::PointXYZI> &arranged,
            pcl::PointCloud<pcl::PointXYZI> &complement);

    pcl::PointCloud<pcl::PointXYZI> ground_viz; ///< 用于可视化的地面点云（在pcs_v2中显示）
    
    // ==================== 版本3算法 ====================
    
    /**
     * @brief 比较VoI并恢复地面点，带阻挡检测（版本3）
     * @param frame 当前帧号
     * @description 实现ERASOR算法的第三版本，增加了阻挡检测功能，更加保守
     */
    void compare_vois_and_revert_ground_w_block(int frame);

    /**
     * @brief 检查附近是否有动态对象
     * @param r_pod_selected 选择的R-POD数据
     * @param r_target 目标径向索引
     * @param theta_target 目标角度索引
     * @param r_range 径向搜索范围
     * @param theta_range 角度搜索范围
     * @return 如果附近有动态对象返回true，否则返回false
     * @description 在指定范围内检查是否存在动态对象
     */
    bool is_dynamic_obj_close(R_POD &r_pod_selected, int r_target, int theta_target, int r_range, int theta_range);

    // ==================== 调试和输出方法 ====================
    
    /**
     * @brief 获取被拒绝的异常点
     * @param map_rejected 输出的被拒绝地图点
     * @param curr_rejected 输出的被拒绝当前点
     * @description 提取在处理过程中被识别为异常的点云
     */
    void get_outliers(
            pcl::PointCloud<pcl::PointXYZI> &map_rejected,
            pcl::PointCloud<pcl::PointXYZI> &curr_rejected);

    // ==================== 公共数据成员 ====================
    
    pcl::PointCloud<pcl::PointXYZI> debug_curr_rejected;  ///< 调试用：被拒绝的当前点云
    pcl::PointCloud<pcl::PointXYZI> debug_map_rejected;   ///< 调试用：被拒绝的地图点云
    pcl::PointCloud<pcl::PointXYZI> map_complement;       ///< 地图补充点云

    R_POD r_pod_map;      ///< 地图的R-POD表示
    R_POD r_pod_curr;     ///< 当前点云的R-POD表示
    R_POD r_pod_selected; ///< 选择的R-POD表示

    /**
     * @brief 获取最大检测距离
     * @return 最大检测距离值
     */
    double get_max_range();

private:
    std::vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};

    ros::NodeHandle nh;
    ros::Publisher  pub_map_rejected;
    ros::Publisher  pub_curr_rejected;
    ros::Publisher  pub_map_init, pub_curr_init;
    ros::Publisher  pub_map_marker, pub_curr_marker, pub_viz_bin_marker;
    ros::Publisher  pub_ground, pub_arranged;

    double max_r;
    int    num_rings;
    int    num_sectors;
    int    num_lowest_pts;

    double ring_size;
    double sector_size;

    double min_h;
    double max_h;
    double scan_ratio_threshold;
    double th_bin_max_h;
    int    minimum_num_pts;
    double rejection_ratio;
    double map_voxel_size_;

    double xy2theta(const double &x, const double &y);

    double xy2radius(const double &x, const double &y);

    void init(R_POD &r_pod);

    void clear_bin(Bin &bin);

    void clear(pcl::PointCloud<pcl::PointXYZI> &pt_cloud);

    pcl::PointCloud<pcl::PointXYZI> piecewise_ground_, non_ground_;
    pcl::PointCloud<pcl::PointXYZI> ground_pc_, non_ground_pc_;

    void pt2r_pod(const pcl::PointXYZI &pt, Bin &bin);

    void voi2r_pod(const pcl::PointCloud<pcl::PointXYZI> &src, R_POD &r_pod);

    void voi2r_pod(
            const pcl::PointCloud<pcl::PointXYZI> &src, R_POD &r_pod,
            pcl::PointCloud<pcl::PointXYZI> &complement);

    void viz_pseudo_occupancy();


    // ------ Ground extraction ------------------------
    double th_dist_; // params!
    int    iter_groundfilter_; // params!
    int    num_lprs_;
    double th_seeds_heights_;

    Eigen::MatrixXf normal_;
    double          th_dist_d_, d_;

    void estimate_plane_(const pcl::PointCloud<pcl::PointXYZI> &ground);

    void extract_initial_seeds_(
            const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
            pcl::PointCloud<pcl::PointXYZI> &init_seeds);

    void extract_ground(
            const pcl::PointCloud<pcl::PointXYZI> &src,
            pcl::PointCloud<pcl::PointXYZI> &dst,
            pcl::PointCloud<pcl::PointXYZI> &outliers);


    bool has_dynamic(Bin &bin);

    void merge_bins(const Bin &src1, const Bin &src2, Bin &dst);

    void r_pod2pc(const R_POD &sc, pcl::PointCloud<pcl::PointXYZI> &pc);


    geometry_msgs::PolygonStamped set_polygons(int r_idx, int theta_idx, int num_split = 3);
};


