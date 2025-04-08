#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Publishers
ros::Publisher imu_pub;
ros::Publisher scan_pub;

// Callback for IMU data
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu new_imu_msg = *msg;
    // Modify or process the IMU data if needed
    imu_pub.publish(new_imu_msg);
}

// Callback for LaserScan data
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan new_scan_msg = *msg;
    new_scan_msg.header.stamp = ros::Time::now() - ros::Duration(0.01);
    // printf("Received scan data with %zu ranges\n", msg->ranges.size());
    // sensor_msgs::LaserScan new_scan_msg;
    // for (size_t i = 0; i < msg->ranges.size() - 1; ++i) {
    //     new_scan_msg.ranges.push_back(msg->ranges[i]);
    // }
    // new_scan_msg.header = msg->header;
    // new_scan_msg.angle_min = msg->angle_min;
    // new_scan_msg.angle_max = msg->angle_max;
    // new_scan_msg.angle_increment = msg->angle_increment;
    // new_scan_msg.time_increment = msg->time_increment;
    // Modify or process the LaserScan data if needed
    scan_pub.publish(new_scan_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_scan_republisher");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scanCallback);

    // Publishers
    imu_pub = nh.advertise<sensor_msgs::Imu>("/device/imu/data", 1);
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/device/lidar/scan", 1);

    ros::spin();
    return 0;
}