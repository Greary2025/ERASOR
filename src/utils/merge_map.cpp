/**
 * @file merge_map.cpp
 * @brief 点云地图合并工具
 * @author ERASOR团队
 * @description 该文件实现了多个点云地图的合并功能，支持保持语义标签的体素化处理
 */

// ROS核心库
#include <ros/ros.h>

// PCL点云处理库
#include <pcl/common/transforms.h>          // 点云变换
#include <pcl_conversions/pcl_conversions.h> // PCL与ROS消息转换
#include <pcl/point_types.h>                 // 点云数据类型
#include <pcl/io/pcd_io.h>                   // PCD文件读写
#include <pcl/point_cloud.h>                 // 点云基础类
#include <pcl/kdtree/kdtree_flann.h>         // KD树最近邻搜索
#include <pcl/common/transforms.h>           // 点云坐标变换
#include <pcl/io/pcd_io.h>                   // PCD文件操作

// 自定义工具库
#include <unavlib/convt.h>                   // 数据转换工具
#include <unavlib/others.h>                  // 其他实用工具

// 标准库
#include <string>                            // 字符串处理
#include <map>                               // 映射容器
#include <vector>                            // 动态数组

using namespace unavlib;                     // 使用自定义命名空间
using namespace std;                         // 使用标准命名空间

/**
 * @brief 保持语义标签的体素化处理函数
 * @param src 输入点云指针
 * @param dst 输出点云引用
 * @param leaf_size 体素大小（米）
 * @description 该函数对点云进行体素化下采样，同时保持原始点的强度值（语义标签）
 */
void erasor_utils::voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
                          pcl::PointCloud<pcl::PointXYZI>& dst,
                          double leaf_size){
  pcl::PointCloud<pcl::PointXYZI> tmp, output;              // 临时和输出点云

  // 第一步：标准体素化处理
  static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;       // 静态体素滤波器
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_filter.setInputCloud(src);                          // 设置输入点云
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size); // 设置体素大小
  voxel_filter.filter(*ptr_voxelized);                      // 执行体素化滤波
  tmp = *ptr_voxelized;                                      // 保存体素化结果

  // 第二步：通过最近邻搜索恢复原始强度值（语义标签）
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;                  // KD树搜索结构
  kdtree.setInputCloud (src);                               // 设置原始点云为搜索目标

  int K = 1;                                                // 搜索最近的1个点
  std::vector<int> pointIdxNKNSearch(K);                    // 最近邻点索引
  std::vector<float> pointNKNSquaredDistance(K);            // 最近邻距离平方
  
  // 为每个体素化后的点找到原始点云中最近的点，并更新强度值
  for (const auto &pt: tmp.points){
    if ( kdtree.nearestKSearch (pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
          auto updated = pt;                                 // 复制体素化后的点
          // 用原始点的强度值替换体素化后的平均强度值
          updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
          output.points.push_back(updated);                  // 添加到输出点云
     }
  }
  dst = output;                                              // 设置输出结果
}

/**
 * @brief 主函数 - 点云地图合并程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 * @description 该程序用于合并多个点云地图文件，支持不同的合并策略
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "merge_map");                      // 初始化ROS节点
    std::cout<<"KiTTI MAP-Merge STARTED"<<std::endl;         // 输出启动信息
    ros::NodeHandle nodeHandler;                             // 创建ROS节点句柄
    // 合并策略1：合并特定帧范围的地图（475~625 + 824~1450）
    // 该策略用于合并两个不连续的帧范围生成的地图
//    std::string s0 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/05_475_to_625_w_interval1_voxel_0.200000.pcd";
//    std::string s1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/05_824_to_1450_w_interval1_voxel_0.200000.pcd";
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr0(new pcl::PointCloud<pcl::PointXYZI>); // 第一个地图点云
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr1(new pcl::PointCloud<pcl::PointXYZI>); // 第二个地图点云

//    datahandle3d::load_pcd(s0, ptr0);                        // 加载第一个地图
//    datahandle3d::load_pcd(s1, ptr1);                        // 加载第二个地图
//    pcl::PointCloud<pcl::PointXYZI> mapOut;                  // 输出合并地图
//    cout<<"src0 : "<<ptr0->points.size()<<endl;              // 输出第一个地图点数
//    cout<<"src1 : "<<ptr1->points.size()<<endl;              // 输出第二个地图点数

    // 合并策略2：预测结果合并
    // 该策略用于合并多个预测处理后的地图片段
//    std::string s0 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/05_25_to_475_w_interval1_voxel_0.200000.pcd";   // 帧25-475
//    std::string s1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/05_625_to_825_w_interval1_voxel_0.200000.pcd";  // 帧625-825
//    std::string s2 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/05_1450_to_1900_w_interval1_voxel_0.200000.pcd"; // 帧1450-1900
//    std::string s3 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/partial_05_475_625_824_1450_result.pcd";      // 部分处理结果
//    std::string s4 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/partial_05_1900_to_2350.pcd";                // 帧1900-2350
//    std::string s5 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/partial_05_2350_2670.pcd";                 // 帧2350-2670

//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr0(new pcl::PointCloud<pcl::PointXYZI>); // 地图片段1
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr1(new pcl::PointCloud<pcl::PointXYZI>); // 地图片段2
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr2(new pcl::PointCloud<pcl::PointXYZI>); // 地图片段3
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr3(new pcl::PointCloud<pcl::PointXYZI>); // 地图片段4
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr4(new pcl::PointCloud<pcl::PointXYZI>); // 地图片段5
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr5(new pcl::PointCloud<pcl::PointXYZI>); // 地图片段6


//    datahandle3d::load_pcd(s0, ptr0);                        // 加载地图片段1
//    datahandle3d::load_pcd(s1, ptr1);                        // 加载地图片段2
//    datahandle3d::load_pcd(s2, ptr2);                        // 加载地图片段3
//    datahandle3d::load_pcd(s3, ptr3);                        // 加载地图片段4
//    datahandle3d::load_pcd(s4, ptr4);                        // 加载地图片段5
//    datahandle3d::load_pcd(s5, ptr5);                        // 加载地图片段6
//    pcl::PointCloud<pcl::PointXYZI> tmp0, tmp1, tmp2, tmpOut, mapOut; // 临时和输出点云

//    cout<<"src0 : "<<ptr0->points.size()<<endl;              // 输出片段1点数
//    cout<<"src1 : "<<ptr1->points.size()<<endl;              // 输出片段2点数
//    cout<<"src2 : "<<ptr2->points.size()<<endl;              // 输出片段3点数
//    cout<<"src3 : "<<ptr3->points.size()<<endl;              // 输出片段4点数
//    cout<<"src4 : "<<ptr4->points.size()<<endl;              // 输出片段5点数
//    cout<<"src5 : "<<ptr5->points.size()<<endl;              // 输出片段6点数
//    tmp0 = *ptr0 + *ptr1;                                    // 合并片段1和2
//    tmp1 = *ptr2 + *ptr3;                                    // 合并片段3和4
//    tmp2 = *ptr4 + *ptr5;                                    // 合并片段5和6

//    tmpOut = tmp0 + tmp1;                                    // 合并前两组
//    tmpOut = tmpOut + tmp2;                                  // 合并所有片段
//    cout<<"tmpOut : "<<tmpOut.points.size()<<endl;          // 输出最终合并点数

    // 合并策略3：原始材料合并
    // 该策略用于合并原始（未经ERASOR处理）的地图片段
//    std::string s0 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_25_to_475_w_interval1_voxel_0.200000.pcd";   // 原始帧25-475
//    std::string s1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_475_to_625_w_interval1_voxel_0.200000.pcd";  // 原始帧475-625
//    std::string s2 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_625_to_825_w_interval1_voxel_0.200000.pcd";  // 原始帧625-825
//    std::string s3 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_824_to_1450_w_interval1_voxel_0.200000.pcd"; // 原始帧824-1450
//    std::string s4 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_1450_to_1900_w_interval1_voxel_0.200000.pcd"; // 原始帧1450-1900
//    std::string s5 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_1900_to_2350_w_interval1_voxel_0.200000.pcd"; // 原始帧1900-2350
//    std::string s6 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_2350_to_2670_w_interval2_voxel_0.200000.pcd"; // 原始帧2350-2670

//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr0(new pcl::PointCloud<pcl::PointXYZI>); // 原始地图片段1
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr1(new pcl::PointCloud<pcl::PointXYZI>); // 原始地图片段2
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr2(new pcl::PointCloud<pcl::PointXYZI>); // 原始地图片段3
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr3(new pcl::PointCloud<pcl::PointXYZI>); // 原始地图片段4
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr4(new pcl::PointCloud<pcl::PointXYZI>); // 原始地图片段5
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr5(new pcl::PointCloud<pcl::PointXYZI>); // 原始地图片段6
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr6(new pcl::PointCloud<pcl::PointXYZI>); // 原始地图片段7


//    datahandle3d::load_pcd(s0, ptr0);                        // 加载原始片段1
//    datahandle3d::load_pcd(s1, ptr1);                        // 加载原始片段2
//    datahandle3d::load_pcd(s2, ptr2);                        // 加载原始片段3
//    datahandle3d::load_pcd(s3, ptr3);                        // 加载原始片段4
//    datahandle3d::load_pcd(s4, ptr4);                        // 加载原始片段5
//    datahandle3d::load_pcd(s5, ptr5);                        // 加载原始片段6
//    datahandle3d::load_pcd(s6, ptr6);                        // 加载原始片段7
//    pcl::PointCloud<pcl::PointXYZI> tmp0, tmp1, tmp2, tmpOut, mapOut; // 临时和输出点云

//    cout<<"src0 : "<<ptr0->points.size()<<endl;              // 输出原始片段1点数
//    cout<<"src1 : "<<ptr1->points.size()<<endl;              // 输出原始片段2点数
//    cout<<"src2 : "<<ptr2->points.size()<<endl;              // 输出原始片段3点数
//    cout<<"src3 : "<<ptr3->points.size()<<endl;              // 输出原始片段4点数
//    cout<<"src4 : "<<ptr4->points.size()<<endl;              // 输出原始片段5点数
//    cout<<"src5 : "<<ptr5->points.size()<<endl;              // 输出原始片段6点数
//    cout<<"src6 : "<<ptr6->points.size()<<endl;              // 输出原始片段7点数
//    tmp0 = *ptr0 + *ptr1;                                    // 合并原始片段1和2
//    tmp1 = *ptr2 + *ptr3;                                    // 合并原始片段3和4
//    tmp2 = *ptr4 + *ptr5;                                    // 合并原始片段5和6
//    tmp2 = tmp2 + *ptr6;                                     // 添加原始片段7

//    tmpOut = tmp0 + tmp1;                                    // 合并前两组
//    tmpOut = tmpOut + tmp2;                                  // 合并所有原始片段
//    cout<<"tmpOut : "<<tmpOut.points.size()<<endl;          // 输出最终合并点数
    // 合并策略4：当前使用的合并方案
    // 合并最终预测结果和部分处理结果
    std::string s0 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/merge_final_05_prediction.pcd";        // 最终预测地图
    std::string s1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/partial_05_475_625_824_1450_result_v2.pcd"; // 部分处理结果v2

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr0(new pcl::PointCloud<pcl::PointXYZI>); // 预测地图点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr1(new pcl::PointCloud<pcl::PointXYZI>); // 部分结果点云

    // 加载点云数据
    datahandle3d::load_pcd(s0, ptr0);                        // 加载预测地图
    datahandle3d::load_pcd(s1, ptr1);                        // 加载部分结果
    pcl::PointCloud<pcl::PointXYZI> tmp0, tmp1, tmp2, tmpOut, mapOut; // 临时和输出点云

    cout<<"src0 : "<<ptr0->points.size()<<endl;              // 输出预测地图点数
    cout<<"src1 : "<<ptr1->points.size()<<endl;              // 输出部分结果点数
    tmpOut = *ptr0 + *ptr1;                                  // 合并两个点云
    cout<<"tmpOut : "<<tmpOut.points.size()<<endl;          // 输出合并后点数

    // 体素化处理以减少点云密度并保持语义标签
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    *ptr_src = tmpOut;                                       // 复制合并后的点云
    cout<<"Voxelizing..."<<endl;                             // 输出体素化提示
    erasor_utils::voxelize_preserving_labels(ptr_src, mapOut, 0.2); // 执行保持标签的体素化（0.2米体素）

    // 设置输出点云的结构信息
    mapOut.width = mapOut.points.size();                    // 设置点云宽度（点数）
    mapOut.height = 1;                                      // 设置点云高度（无序点云为1）

    // 保存最终合并和体素化后的地图
    pcl::io::savePCDFileASCII("/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/merge_final_05_prediction_v2.pcd", mapOut);
//    pcl::io::savePCDFileASCII("/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/merge_final_raw.pcd", mapOut); // 原始合并结果保存路径（已注释）
    std::cout<<"Complete to save"<<std::endl;               // 输出保存完成信息

    return 0;                                                // 程序正常退出
}
