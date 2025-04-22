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
<<<<<<< HEAD
=======
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)
>>>>>>> cbd22fdedd92d6f3749e3802b376c25bd23648dc
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

<<<<<<< HEAD
// Global variables
float MAX_LIN_VEL = 0.5f; // in m/s
float MAX_ANG_VEL = 0.25f; // in rad/s
float MAX_LIN_ACC = 0.01f; // in m/s^2
float MAX_ANG_ACC = 1.0f; // in rad/s^2
float Kp = 0.1f;
float Ki = 0.0f;
float Kd = 0.0f;
float Kp_angular = 0.1f;
float Ki_angular = 0.0f;
float Kd_angular = 0.0f;
float tf_lidar2base_x = 0.0f;
float tf_lidar2base_y = 0.0f;
float tf_lidar2base_theta = 0.0f;

pose_t initial_pose = { 0.0, 0.0, 0.0 };

pose_t robot_pose = { 0.0, 0.0, 0.0 }; // x, y, theta
pose_t robot_vel = { 0.0, 0.0, 0.0 }; // vx, vy, omega
pose_t robot_odom = { 0.0, 0.0, 0.0 };
pose_t robot_odom_dot = { 0.0, 0.0, 0.0 };
uint16_t robot_state = 0;
uint8_t controlled_by = 0; // 0: keyboard, 1: joystick

uint8_t use_slam = 0;
uint8_t use_sim = 0;
uint8_t use_gmapping = 0;

float initial_imu_yaw = 0.0f;
bool imu_initialized = false;
float initial_pose_theta = 0.0f;
float last_safe_theta = 0.0f;

pose_t goal_pose;

std::vector<point2d_t> lidar_data;

axis_t axis_left;
axis_t axis_right;
button_t buttons;
=======
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
>>>>>>> cbd22fdedd92d6f3749e3802b376c25bd23648dc

ros::Time time_control;

// ROS objects
<<<<<<< HEAD
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
=======
ros::Timer timer_main;
ros::Subscriber sub_joy;
ros::Subscriber sub_imu;
ros::Subscriber sub_lidar;
ros::Subscriber sub_encoder;
ros::Subscriber sub_amcl_pose;
ros::Subscriber sub_goal_pose;
ros::Subscriber sub_init_pose;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_robot_pose;
ros::Publisher pub_robot_odom;
ros::Publisher pub_marker;

tf::TransformBroadcaster* tf_broadcaster;
tf::TransformListener* tf_listener;

geometry_msgs::PoseWithCovarianceStamped amcl_pose;

// Function prototypes
>>>>>>> 8a04e197ce205f4bd173596cf633669370180b3a
void timer_callback(const ros::TimerEvent&);

// Subscriber callbacks
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void encoder_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
<<<<<<< HEAD
void odom_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg);
void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
=======
void goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
>>>>>>> 8a04e197ce205f4bd173596cf633669370180b3a

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