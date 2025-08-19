/**
 * @file pub_marker.cpp
 * @brief RViz可视化标记发布器
 * @author ERASOR团队
 * @description 该文件实现了一个ROS节点，用于在RViz中发布立方体列表标记，
 *              主要用于可视化调试和演示目的
 */

// ROS核心库
#include <ros/ros.h>
#include <geometry_msgs/Point.h>          // 几何点消息类型
#include <visualization_msgs/Marker.h>    // 可视化标记消息类型
#include <std_msgs/ColorRGBA.h>          // 颜色消息类型

// 自定义工具库
#include <unavlib/convt.h>               // 转换工具
#include <unavlib/others.h>              // 其他实用工具

// 标准库
#include <string>
#include <map>
#include <vector>

/**
 * @brief RViz可视化标记发布器
 * @description 用于发布可视化标记到RViz进行显示
 */
ros::Publisher  vizPublisher;

using namespace unavlib;


/**
 * @brief 主函数 - 可视化标记发布器
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态码
 * @description 创建并发布立方体列表标记到RViz进行可视化显示
 */
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "vizmarker");

    // 创建节点句柄
    ros::NodeHandle nodeHandler;

    // 创建可视化标记发布器，话题名为"/marker"，队列大小为100
    vizPublisher = nodeHandler.advertise<visualization_msgs::Marker>("/marker", 100);

    // 创建立方体列表标记消息
    visualization_msgs::Marker cube_list;
    cube_list.header.frame_id = "map";                           // 设置坐标系为地图坐标系
    cube_list.header.stamp = ros::Time::now();                   // 设置时间戳为当前时间
    cube_list.ns = "cubes";                                      // 设置命名空间为"cubes"
    cube_list.action = visualization_msgs::Marker::ADD;          // 设置动作为添加标记
    cube_list.pose.orientation.w = 1.0;                         // 设置四元数方向（无旋转）
    cube_list.type = visualization_msgs::Marker::CUBE_LIST;      // 设置标记类型为立方体列表
    
    // 设置立方体尺寸（米）
    cube_list.scale.x = 10.0;                                   // X方向尺寸：10米
    cube_list.scale.y = 10.0;                                   // Y方向尺寸：10米
    cube_list.scale.z = 2.0;                                    // Z方向尺寸：2米

    // 设置默认颜色（蓝色，半透明）
    cube_list.color.r = 0.0;                                    // 红色分量：0
    cube_list.color.g = 0.0;                                    // 绿色分量：0
    cube_list.color.b = 1.0;                                    // 蓝色分量：1（纯蓝色）
    cube_list.color.a = 0.3;                                    // 透明度：0.3（半透明）
    cube_list.id = 3;                                           // 标记ID：3

    // 创建几何点和颜色变量
    geometry_msgs::Point p;
    std_msgs::ColorRGBA test;

    // 添加第一个立方体（位置：(10, 10, 10)，颜色：洋红色）
    p.x = 10;                                                    // X坐标：10米
    p.y = 10;                                                    // Y坐标：10米
    p.z = 10;                                                    // Z坐标：10米

    test.r = 1.0; test.g = 0.0; test.b = 1.0; test.a = 1.0;    // 洋红色，不透明

    cube_list.points.push_back(p);                              // 添加点到立方体列表
    cube_list.colors.push_back(test);                           // 添加对应颜色

    // 添加第二个立方体（位置：(0, 0, 0)，颜色：青色）
    p.x = 0;                                                     // X坐标：0米（原点）
    p.y = 0;                                                     // Y坐标：0米
    p.z = 0;                                                     // Z坐标：0米

    test.r = 0.0; test.g = 1.0; test.b = 1.0; test.a = 1.0;    // 青色，不透明

    cube_list.points.push_back(p);                              // 添加点到立方体列表
    cube_list.colors.push_back(test);                           // 添加对应颜色

    // 设置发布频率为2Hz
    ros::Rate loop_rate(2);
    
    // 主循环：持续发布可视化标记
    while (ros::ok())
    {
      vizPublisher.publish(cube_list);                          // 发布立方体列表标记
      ros::spinOnce();                                          // 处理ROS回调
      loop_rate.sleep();                                        // 按设定频率休眠
    }
    
    return 0;                                                   // 程序正常退出
}
