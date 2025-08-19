/**
 * @file pub_polygon.cpp
 * @brief RViz多边形可视化发布器
 * @author ERASOR团队
 * @description 该文件实现了一个ROS节点，用于在RViz中发布多边形数组标记，
 *              主要用于可视化几何形状和区域边界
 */

// ROS核心库
#include <ros/ros.h>
#include <geometry_msgs/Point.h>              // 几何点消息类型
#include <geometry_msgs/PolygonStamped.h>     // 带时间戳的多边形消息类型
#include <geometry_msgs/Polygon.h>            // 多边形消息类型
#include <geometry_msgs/Point32.h>            // 32位几何点消息类型
#include <visualization_msgs/Marker.h>        // 可视化标记消息类型
#include <std_msgs/ColorRGBA.h>              // 颜色消息类型
#include <jsk_recognition_msgs/PolygonArray.h> // JSK多边形数组消息类型

// 自定义工具库
#include <unavlib/convt.h>                   // 转换工具
#include <unavlib/others.h>                  // 其他实用工具

// 标准库
#include <string>
#include <map>
#include <vector>

/**
 * @brief RViz多边形可视化发布器
 * @description 用于发布多边形数组标记到RViz进行显示
 */
ros::Publisher  vizPublisher;

using namespace unavlib;


/**
 * @brief 主函数 - 多边形可视化发布器
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态码
 * @description 创建并发布多边形数组到RViz进行可视化显示
 */
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "vizmarker");

    // 创建节点句柄
    ros::NodeHandle nodeHandler;

    // 创建多边形数组发布器，话题名为"/poly_marker"，队列大小为100
    vizPublisher = nodeHandler.advertise<jsk_recognition_msgs::PolygonArray>("/poly_marker", 100);

    // 创建多边形数组消息
    jsk_recognition_msgs::PolygonArray poly_list;
    poly_list.header.frame_id = "/world";                        // 设置坐标系为世界坐标系
    poly_list.header.stamp = ros::Time::now();                   // 设置时间戳为当前时间

    // 创建多边形元素
    geometry_msgs::PolygonStamped polygons;
    polygons.header = poly_list.header;                          // 继承多边形数组的头信息

    // 创建32位几何点变量
    geometry_msgs::Point32 point;

    // 定义多边形的顶点（构成一个不规则五边形）
    point.x = 2.0; point.y = 0.0; point.z = 5.0;                // 第一个顶点：(2, 0, 5)
    polygons.polygon.points.push_back(point);
    polygons.polygon.points.push_back(point);                   // 重复添加同一点（可能用于闭合）
    
    point.x = 4.0; point.y = 0.0; point.z = 5.0;                // 第二个顶点：(4, 0, 5)
    polygons.polygon.points.push_back(point);
    
    point.x = 0.0; point.y = 4.0; point.z = 5.0;                // 第三个顶点：(0, 4, 5)
    polygons.polygon.points.push_back(point);

    point.x = 0.0; point.y = 2.0; point.z = 5.0;                // 第四个顶点：(0, 2, 5)
    polygons.polygon.points.push_back(point);
    
    // 将构建的多边形添加到多边形数组中
    poly_list.polygons.push_back(polygons);

    // 添加多边形的置信度/可能性值（0.0表示低置信度）
    poly_list.likelihood.push_back(0.0);

    // 设置发布频率为2Hz
    ros::Rate loop_rate(2);
    
    // 主循环：持续发布多边形可视化标记
    while (ros::ok())
    {
      static int cnt;                                            // 静态计数器，记录发布次数
      vizPublisher.publish(poly_list);                          // 发布多边形数组
      std::cout<<"pub! "<<++cnt<<std::endl;                     // 输出发布计数信息
      ros::spinOnce();                                          // 处理ROS回调
      loop_rate.sleep();                                        // 按设定频率休眠
    }
    
    return 0;                                                   // 程序正常退出
}
