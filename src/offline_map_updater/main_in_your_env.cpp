/**
 * @file main_in_your_env.cpp
 * @brief ERASOR自定义环境主程序
 * @author shapelim (创建于 21. 10. 18.)
 * @description 用于在用户自定义环境中运行ERASOR算法的主程序
 *              支持从PCD文件和位姿数据构建静态地图
 */

// ==================== 头文件包含 ====================
#include "tools/erasor_utils.hpp"                              ///< ERASOR工具函数
#include <boost/format.hpp>                                   ///< Boost格式化库
#include <cstdlib>                                            ///< 标准库函数
#include <erasor/OfflineMapUpdater.h>                         ///< ERASOR离线地图更新器

// ==================== 全局变量定义 ====================
string DATA_DIR;                                              ///< 数据目录路径
int INTERVAL, INIT_IDX;                                       ///< 帧间隔和初始索引
float VOXEL_SIZE;                                             ///< 体素大小
bool STOP_FOR_EACH_FRAME;                                     ///< 是否在每帧停止等待用户输入
std::string filename = "/staticmap_via_erasor.pcd";           ///< 静态地图输出文件名

// ==================== 类型别名定义 ====================
using PointType = pcl::PointXYZI;                            ///< 点云数据类型


/**
 * @brief 按分隔符分割字符串并转换为浮点数向量
 * @param input 输入字符串
 * @param delimiter 分隔符字符
 * @return 分割后的浮点数向量
 * @description 将输入字符串按指定分隔符分割，并将每个子字符串转换为浮点数
 */
vector<float> split_line(string input, char delimiter) {
    vector<float> answer;                                         ///< 存储分割结果的向量
    stringstream ss(input);                                       ///< 字符串流用于分割
    string temp;                                                  ///< 临时字符串存储每个分割片段

    // 按分隔符逐个读取字符串片段
    while (getline(ss, temp, delimiter)) {
        answer.push_back(stof(temp));                             ///< 转换为浮点数并添加到结果向量
    }
    return answer;
}

/**
 * @brief 从CSV文件加载所有位姿数据
 * @param txt CSV文件路径
 * @param poses 输出的位姿变换矩阵向量
 * @description 从CSV文件中读取位姿数据，转换为4x4变换矩阵
 *              注意：这些位姿已经相对于车体坐标系！
 *              因此，变换矩阵 * 对应点云 = 地图点云
 */
void load_all_poses(string txt, vector<Eigen::Matrix4f >& poses){
    // 这些位姿已经相对于车体坐标系！
    // 因此，tf4x4 * 对应点云 -> 地图点云
    cout<<"Target path: "<< txt<<endl;
    poses.clear();                                                ///< 清空位姿向量
    poses.reserve(2000);                                          ///< 预分配内存空间
    std::ifstream in(txt);                                        ///< 打开CSV文件
    std::string line;                                             ///< 存储每行数据的字符串

    int count = 0;                                                ///< 行计数器
    while (std::getline(in, line)) {
        // 跳过第一行（通常是标题行）
        if (count == 0){
            count++;
            continue;
        }

        vector<float> pose = split_line(line, ',');               ///< 分割CSV行数据

        // 从CSV数据构建变换矩阵
        // 假设CSV格式：[其他数据, x, y, z, qx, qy, qz, qw, 其他数据]
        Eigen::Translation3f ts(pose[2], pose[3], pose[4]);       ///< 平移向量 (x, y, z)
        Eigen::Quaternionf q(pose[8], pose[5], pose[6], pose[7]); ///< 四元数 (w, x, y, z)
        Eigen::Matrix4f tf4x4_cam = Eigen::Matrix4f::Identity();  ///< 初始化为单位矩阵（关键！）
        tf4x4_cam.block<3, 3>(0, 0) = q.toRotationMatrix();      ///< 设置旋转矩阵部分
        tf4x4_cam.block<3, 1>(0, 3) = ts.vector();               ///< 设置平移向量部分
        Eigen::Matrix4f tf4x4_lidar = tf4x4_cam;                 ///< 复制变换矩阵
        
        poses.emplace_back(tf4x4_lidar);                          ///< 添加到位姿向量中
        count++;                                                  ///< 增加计数器
    }
    std::cout<<"Total "<<count<<" poses are loaded"<<std::endl; ///< 输出加载的位姿总数
}

/**
 * @brief 主函数 - ERASOR自定义环境处理入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 * @description 初始化ROS节点，加载数据，运行ERASOR算法构建静态地图
 */
int main(int argc, char **argv)
{
    // ==================== ROS初始化 ====================
    ros::init(argc, argv, "erasor_in_your_env");              ///< 初始化ROS节点
    ros::NodeHandle nh;                                        ///< 创建节点句柄
    erasor::OfflineMapUpdater updater = erasor::OfflineMapUpdater(); ///< 创建ERASOR离线地图更新器

    // ==================== 参数加载 ====================
    nh.param<string>("/data_dir", DATA_DIR, "/");             ///< 数据目录路径
    nh.param<float>("/voxel_size", VOXEL_SIZE, 0.075);        ///< 体素网格大小
    nh.param<int>("/init_idx", INIT_IDX, 0);                  ///< 起始帧索引
    nh.param<int>("/interval", INTERVAL, 2);                  ///< 处理帧间隔
    nh.param<bool>("/stop_for_each_frame", STOP_FOR_EACH_FRAME, false); ///< 是否每帧暂停

    std::string staticmap_path = std::getenv("HOME") + filename; ///< 静态地图保存路径

    // ==================== ROS发布器设置 ====================
    ros::Publisher NodePublisher = nh.advertise<erasor::node>("/node/combined/optimized", 100); ///< 节点数据发布器
    ros::Rate loop_rate(10);                                   ///< 循环频率控制

    // ==================== 目标数据设置 ====================
    /**
     * 设置目标数据
     * 注意：PCD文件位于 `pcd_dir` 目录中
     * 确保第i个位姿对应第i个PCD文件
     *
     * 在我们的示例中，变换结果（位姿 * 点云）直接表示
     * 地图点云的部分区域
     * （即没有额外的变换矩阵，例如lidar2body）
     *
     * 在您自己的情况下，请注意正确设置max_h和min_h！
     */
    cout << "\033[1;32mTarget directory:" << DATA_DIR << "\033[0m" << endl;
    string raw_map_path = DATA_DIR + "/dense_global_map.pcd";     ///< 原始密集全局地图路径
    string pose_path = DATA_DIR + "/poses_lidar2body.csv";        ///< 位姿文件路径
    string pcd_dir = DATA_DIR + "/pcds";                          ///< PCD文件目录
    
    // ==================== 加载位姿数据 ====================
    vector<Eigen::Matrix4f> poses;                                ///< 存储所有位姿的向量
    load_all_poses(pose_path, poses);                             ///< 从CSV文件加载位姿数据

    int N = poses.size();                                          ///< 位姿总数

    // ==================== 点云数据处理循环 ====================
    for (int i = INIT_IDX; i < N; ++i) {                          ///< 遍历所有点云帧
        signal(SIGINT, erasor_utils::signal_callback_handler);    ///< 设置中断信号处理器

        // ==================== 加载点云文件 ====================
        pcl::PointCloud<PointType>::Ptr srcCloud(new pcl::PointCloud<PointType>); ///< 源点云指针
        string pcd_name = (boost::format("%s/%06d.pcd") % pcd_dir % i).str();     ///< 构造PCD文件名（6位数字格式）
        erasor_utils::load_pcd(pcd_name, srcCloud);               ///< 使用工具函数加载PCD文件

        // ==================== 构建ROS节点消息 ====================
        erasor::node node;                                         ///< 创建ERASOR节点消息
        node.header.seq = i;                                      ///< 设置序列号
        node.odom = erasor_utils::eigen2geoPose(poses[i]);        ///< 将Eigen位姿转换为ROS几何位姿
        node.lidar = erasor_utils::cloud2msg(*srcCloud);          ///< 将PCL点云转换为ROS消息格式
        NodePublisher.publish(node);                              ///< 发布节点消息
        ros::spinOnce();                                           ///< 处理ROS回调
        loop_rate.sleep();                                         ///< 控制循环频率

        // ==================== 调试控制 ====================
        if (STOP_FOR_EACH_FRAME) {                                ///< 如果启用逐帧暂停
            cout<< "[Debug]: STOP! Press any button to continue" <<endl; ///< 提示用户按键继续
            cin.ignore();                                          ///< 等待用户输入
        }
    }

    // ==================== 保存静态地图 ====================
    cout << "\033[1;32mSaving static map to " << staticmap_path << "\033[0m" << endl; ///< 输出保存路径信息
    updater.save_static_map(0.2);                             ///< 保存ERASOR处理后的静态地图

    // ==================== 程序结束 ====================
    cout<< "Static map building complete!" << endl;           ///< 输出完成信息

    return 0;                                                  ///< 正常退出程序
}