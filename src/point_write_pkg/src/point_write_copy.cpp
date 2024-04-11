#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

ros::Publisher map_pub;

int pixelSize = 5; 
double ego_x_new = 0.0;
double ego_y_new = 0.0;
double ego_yaw_new = 0.0;
double ego_roll_new = 0.0;
double ego_pitch_new = 0.0;

int map_x = 0;
int map_y = 0;
int map_width;
int map_height;
double map_resolution;
double map_origin_x;
double map_origin_y;
nav_msgs::OccupancyGrid modified_map;

void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& ground_truth)
{
    // 从 ground_truth 消息中获取小车的位置信息
    ego_x_new = ground_truth->pose.pose.position.x - 17;
    ego_y_new = ground_truth->pose.pose.position.y + 7.61;
    std::cout << "x_ " << ego_x_new << std::endl;
    // 四元数求解航向角
    tf2::Quaternion quat(ground_truth->pose.pose.orientation.x, ground_truth->pose.pose.orientation.y, ground_truth->pose.pose.orientation.z, ground_truth->pose.pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(ego_roll_new, ego_pitch_new, ego_yaw_new);
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
{
    // 将ROS PointCloud2消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*input_cloud, pcl_cloud);

    // 创建一个新的点云对象，用于存储处理后的数据 因为原始点云数据修改报错不可以修改
    pcl::PointCloud<pcl::PointXYZ> processed_cloud;
  
    // 对点云数据进行处理，并将占据状态填充到地图中
    for (const auto& point : pcl_cloud.points)
    {
        pcl::PointXYZ processed_point;
        // 将障碍物点云坐标转换为相对于车辆的世界坐标
        processed_point.x = cos(ego_yaw_new) * point.x - sin(ego_yaw_new) * point.y + ego_x_new;
        processed_point.y = sin(ego_yaw_new) * point.x + cos(ego_yaw_new) * point.y + ego_y_new;

        // 计算点在地图中的栅格坐标
        map_x = (processed_point.x * pixelSize) / pixelSize;
        map_y = (processed_point.y * pixelSize) / pixelSize;

        // 检查栅格坐标是否在地图范围内
        if (map_x >= 0 && map_x < map_width && map_y >= 0 && map_y < map_height)
        {
            // 求在地图中的索引
            int index = map_y * map_width + map_x;

            // 将相应的地图栅格设为占据
            modified_map.data[index] = 100; // 100 表示占据

        }
        // 发布修改后的地图消息
        map_pub.publish(modified_map);
    }
    std::cout << "over!" << std::endl;
}

// 回调函数，处理接收到的地图消息
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    // 创建一个新的地图消息用于修改
    modified_map = *map_msg;

    // 获取地图的宽度和分辨率
    map_width = map_msg->info.width;
    map_height = map_msg->info.height;
    map_resolution = map_msg->info.resolution;
    map_origin_x = map_msg->info.origin.position.x;
    map_origin_y = map_msg->info.origin.position.y;
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "point_cloud_to_map");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10); // 设置主循环频率为10Hz


    // 订阅地图话题，指定回调函数
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);

    // 创建ROS发布器并声明在全局范围内，发布processed_map地图话题
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/processed_map", 1);

    // 订阅 ground_truth 话题
    ros::Subscriber sub_ground_truth = nh.subscribe<nav_msgs::Odometry>("/ground_truth", 1, groundTruthCallback);

    // 订阅指定的点云话题
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/points_no_ground", 1, pointCloudCallback);

    // 循环等待
    ros::spin();

    return 0;
}
