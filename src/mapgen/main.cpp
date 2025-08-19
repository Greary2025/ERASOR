/**
 * @file main.cpp
 * @brief ERASOR地图生成器主程序
 * @details 该程序用于从ROS bag文件中读取点云数据，生成全局地图
 *          支持大规模地图构建和实时可视化功能
 * @author ERASOR Team
 */

#include "mapgen.hpp"

/// 地图生成器实例
mapgen mapgenerator;

/// ROS发布器：当前点云数据
ros::Publisher cloudPublisher;
/// ROS发布器：累积地图数据
ros::Publisher mapPublisher;
/// ROS发布器：轨迹路径数据
ros::Publisher pathPublisher;
/// 机器人轨迹路径
nav_msgs::Path path;

using namespace erasor;

/// 数据序列名称（如KITTI序列号）
std::string sequence;
/// 起始时间戳
std::string init_stamp;
/// 结束时间戳
std::string final_stamp;
/// 地图保存路径
std::string save_path;

/// 数据处理间隔
int   interval;
/// 可视化间隔
int   viz_interval;

/// 是否为大规模地图构建模式
bool is_large_scale;

/// 体素化网格大小
float voxelsize;

/**
 * @brief 保存全局地图到PCD文件
 * @details 生成原始地图和处理后地图的文件名，并调用地图生成器保存地图
 *          文件名格式：序列_起始时间_to_结束时间_w_interval间隔_voxel_体素大小[_original].pcd
 */
void saveGlobalMap(){
    /// 构建原始地图文件路径（包含_original后缀）
    std::string original_dir =
                        save_path + "/" + sequence + "_" + init_stamp +
                        "_to_" + final_stamp + "_w_interval" + std::to_string(interval) + "_voxel_" + std::to_string(voxelsize) +
                        "_original.pcd";

    /// 构建处理后地图文件路径
    std::string map_dir = save_path + "/" + sequence + "_" + init_stamp +
                          "_to_" + final_stamp + "_w_interval" + std::to_string(interval) + "_voxel_" + std::to_string(voxelsize) +
                          ".pcd";

    /// 调用地图生成器保存原始地图和处理后地图
    mapgenerator.saveNaiveMap(original_dir, map_dir);
}
/**
 * @brief 保存标志回调函数
 * @param msg 接收到的Float32消息指针
 * @details 当接收到保存标志消息时，触发全局地图保存操作
 *          通常用于手动触发地图保存
 */
void callbackSaveFlag(const std_msgs::Float32::ConstPtr &msg) {
    /// 输出保存标志接收提示
    std::cout << "Flag comes!" << std::endl;
    /// 执行全局地图保存操作
    saveGlobalMap();
}

/**
 * @brief 点云数据回调函数
 * @param msg 接收到的节点消息，包含点云数据和位姿信息
 * @details 处理每一帧点云数据，累积构建全局地图，并进行可视化发布
 *          当达到结束时间戳时自动保存地图
 */
void callbackData(const node msg) {
    /// 设置信号处理器，用于优雅退出
    signal(SIGINT, erasor_utils::signal_callback_handler);
    /// 静态计数器，记录处理的帧数
    static int cnt = 0;
    /// 按可视化间隔输出处理进度
    if ((cnt % viz_interval) == 0){
        std::cout << std::left << setw(nameWidth) << setfill(separator) << "[MAPGEN] " << msg.header.seq << "th frame comes!" << std::endl;
    }

    /// 将当前帧点云累积到全局地图中，同时更新轨迹路径
    mapgenerator.accumPointCloud(msg, path);
    /// 检查是否达到结束时间戳，如果是则保存全局地图
    if (msg.header.seq >= std::stoi(final_stamp)){
        saveGlobalMap();
    }

    /// 可视化处理：按间隔发布点云和地图数据
    if ((cnt % viz_interval) == 0){
        /// 创建当前帧点云指针
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCurr(new pcl::PointCloud<pcl::PointXYZI>());
        /// 创建累积地图点云指针
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZI>());

        /// 从地图生成器获取当前点云和累积地图
        mapgenerator.getPointClouds(cloudMap, cloudCurr);
        /// 发布当前帧点云数据
        cloudPublisher.publish(erasor_utils::cloud2msg(*cloudCurr));
        /// 发布累积地图数据
        mapPublisher.publish(erasor_utils::cloud2msg(*cloudMap));
        /// 发布轨迹路径数据
        pathPublisher.publish(path);
    }
    /// 增加帧计数器
    cnt++;
}

/**
 * @brief 解析ROS bag文件名
 * @param rosbag_name ROS bag文件名（引用传递，会被修改）
 * @return std::vector<std::string> 解析后的字符串数组
 * @details 按下划线分隔符解析文件名，提取序列号、时间戳、间隔等信息
 *          例如："00_000000_to_004540_w_interval10" -> ["00", "000000", "to", "004540", "w", "interval10"]
 */
std::vector<std::string> parseRosbagName(std::string& rosbag_name){
    /// 设置分隔符为下划线
    std::string delimiter = "_";
    /// 查找位置索引
    size_t pos = 0;
    /// 存储解析结果的字符串数组
    std::vector<std::string> string_parsed;
    /// 临时存储分割出的字符串片段
    std::string token;
    /// 循环查找分隔符并分割字符串
    while ((pos = rosbag_name.find(delimiter)) != std::string::npos) {
        /// 提取当前分隔符前的子字符串
        token = rosbag_name.substr(0, pos);
        /// 将提取的字符串添加到结果数组
        string_parsed.push_back(token);
        /// 从原字符串中删除已处理的部分（包括分隔符）
        rosbag_name.erase(0, pos + delimiter.length());
    }
    /// 返回解析后的字符串数组
    return string_parsed;

}

/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态码
 * @details ERASOR地图生成器的主入口函数，负责初始化ROS节点、加载参数、
 *          设置发布器和订阅器，并启动消息循环处理
 */
int main(int argc, char **argv) {
    /// 初始化ROS节点，节点名为"merger"
    ros::init(argc, argv, "merger");
    /// 创建ROS节点句柄
    ros::NodeHandle nodeHandler;
    /// 输出程序启动提示
    std::cout << "KiTTI MAPGEN STARTED" << std::endl;

    /// 目标ROS bag文件名
    std::string target_rosbag;

    /// 从ROS参数服务器加载体素化网格大小，默认0.05米
    nodeHandler.param("/map/voxelsize", voxelsize, (float) 0.05);
    /// 从ROS参数服务器加载目标ROS bag文件名
    nodeHandler.param<std::string>("/map/target_rosbag", target_rosbag, "/");
    /// 从ROS参数服务器加载地图保存路径
    nodeHandler.param<std::string>("/map/save_path", save_path, "/");
    /// 从ROS参数服务器加载可视化间隔，默认10帧
    nodeHandler.param<int>("/map/viz_interval", viz_interval, 10);

    /// 以下参数用于大规模地图构建
    /// 因为ROS发布的点云数据量限制在1GB以下
    /// 这样设置是为了减少计算负担
    nodeHandler.param<bool>("/large_scale/is_large_scale", is_large_scale, false);

    /// 解析ROS bag文件名，提取序列信息
    auto name_parsed = parseRosbagName(target_rosbag);
    /// 提取序列号（索引0）
    sequence = name_parsed[0];
    /// 提取起始时间戳（索引1）
    init_stamp = name_parsed[1];
    /// 提取结束时间戳（索引3）
    final_stamp = name_parsed[3];
    /// 提取处理间隔（索引6，需要转换为整数）
    interval = std::stoi(name_parsed[6]);

    /// 设置地图生成器的参数
    mapgenerator.setValue(save_path, voxelsize, sequence, init_stamp, final_stamp, interval, is_large_scale);
    /// 创建地图数据发布器
    mapPublisher   = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapgen/map", 100);
    /// 创建当前点云数据发布器
    cloudPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapgen/curr", 100);
    /// 创建轨迹路径发布器
    pathPublisher  = nodeHandler.advertise<nav_msgs::Path>("/path", 100);

    /// 订阅优化后的节点数据（包含点云和位姿信息）
    ros::Subscriber subData = nodeHandler.subscribe<node>("/node/combined/optimized", 3000, callbackData);
    /// 订阅保存标志消息
    ros::Subscriber subSaveFlag = nodeHandler.subscribe<std_msgs::Float32>("/saveflag", 1000, callbackSaveFlag);
    /// 启动ROS消息循环，等待并处理消息
    ros::spin();

    /// 程序正常退出
    return 0;
}
