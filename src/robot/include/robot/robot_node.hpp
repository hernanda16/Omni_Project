#ifndef ROBOT_NODE_HPP
#define ROBOT_NODE_HPP

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <vector>

// Control modes
#define KEYBOARD 0
#define JOYSTICK 1

// Operation modes
#define MANUAL 0
#define AUTO 1
#define DEBUG 2

// Conversion constants
#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)
#define ENC2CM 0.09311709475
#define ENC2DEG 0.399185185185

// Struct definitions
typedef struct {
    float x;
    float y;
} axis_t;

typedef struct {
    int8_t x;
    int8_t o;
    int8_t square;
    int8_t triangle;
} button_t;

typedef struct {
    float x;
    float y;
    float theta;
} pose_t;

typedef struct {
    float x;
    float y;
} point2d_t;

// Global parameters
extern float MAX_LIN_VEL;
extern float MAX_ANG_VEL;
extern float MAX_LIN_ACC;
extern float MAX_ANG_ACC;
extern float Kp;
extern float Ki;
extern float Kd;

// LiDAR to base transform
extern float tf_lidar2base_x;
extern float tf_lidar2base_y;
extern float tf_lidar2base_z;
extern float tf_lidar2base_roll;
extern float tf_lidar2base_pitch;
extern float tf_lidar2base_yaw;
extern float tf_lidar2base_theta; // Legacy 2D theta for compatibility

// IMU to base transform
extern float tf_imu2base_x;
extern float tf_imu2base_y;
extern float tf_imu2base_z;
extern float tf_imu2base_roll;
extern float tf_imu2base_pitch;
extern float tf_imu2base_yaw;

// Global state variables
extern pose_t initial_pose;
extern pose_t robot_pose;
extern pose_t robot_vel;
extern pose_t robot_odom;
extern pose_t target_pose;
extern bool have_target;
extern uint16_t robot_state;
extern uint8_t controlled_by;
extern uint8_t use_slam;
extern uint8_t use_amcl;
extern uint8_t use_gmapping;
extern uint8_t use_sim;
extern bool imu_initialized;
extern float initial_imu_yaw;
extern float last_safe_theta;
extern std::vector<point2d_t> lidar_data;

// Control inputs
extern axis_t axis_left;
extern axis_t axis_right;
extern button_t buttons;

// Frame IDs (configurable)
extern std::string map_frame_id;
extern std::string odom_frame_id;
extern std::string base_frame_id;
extern std::string laser_frame_id;
extern std::string imu_frame_id;

// ROS objects
extern ros::Timer timer_main;
extern ros::Subscriber sub_joy;
extern ros::Subscriber sub_imu;
extern ros::Subscriber sub_lidar;
extern ros::Subscriber sub_encoder;
extern ros::Subscriber sub_amcl_pose;
extern ros::Subscriber sub_odom_filter;
extern ros::Subscriber sub_initialpose;
extern ros::Subscriber sub_goal;
extern ros::Publisher pub_cmd_vel;
extern ros::Publisher pub_robot_pose;
extern ros::Publisher pub_robot_odom;
extern ros::Publisher pub_raw_odom;
extern ros::Publisher pub_marker;
extern ros::Publisher pub_markers;
extern ros::Publisher pub_path;
extern ros::Publisher pub_imu;

extern tf::TransformBroadcaster* tf_broadcaster;
extern tf::TransformListener* tf_listener;

// Message storage
extern geometry_msgs::PoseWithCovarianceStamped amcl_pose;
extern sensor_msgs::Imu imu_msg;
extern nav_msgs::Path path_msg;

// Timer callback
void timer_callback(const ros::TimerEvent&);

// Subscriber callbacks
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void encoder_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void odom_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg);
void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

// Robot pose management
void update_robot_pose();
float compute_amcl_trust();
void set_initial_pose(float x, float y, float theta);

// Control functions
void keyboard_handler();
void joystick_handler();
void state_control();
void velocity_control(float vx, float vy, float vtheta);
uint8_t position_control(float x, float y, float theta);

// Utility functions
int8_t kbhit();
void dummy_odom();

// Publication functions
void publish_all();
void publish_tf();
void publish_visualization();

#endif // ROBOT_NODE_HPP