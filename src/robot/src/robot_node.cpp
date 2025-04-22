// Implementation of the remaining functions from robot_node.hpp
#include <robot/robot_node.hpp>

// Global parameters
float MAX_LIN_VEL = 0.5f;
float MAX_ANG_VEL = 0.2f;
float MAX_LIN_ACC = 0.2f;
float MAX_ANG_ACC = 0.1f;
float Kp = 1.0f;
float Ki = 0.0f;
float Kd = 0.0f;
float tf_lidar2base_x = 0.0f;
float tf_lidar2base_y = 0.0f;
float tf_lidar2base_theta = 0.0f;
float tf_lidar2base_z = 0.0f;
float tf_lidar2base_roll = 0.0f;
float tf_lidar2base_pitch = 0.0f;
float tf_lidar2base_yaw = 0.0f;
float tf_imu2base_x = 0.0f;
float tf_imu2base_y = 0.0f;
float tf_imu2base_z = 0.0f;
float tf_imu2base_roll = 0.0f;
float tf_imu2base_pitch = 0.0f;
float tf_imu2base_yaw = 0.0f;

// Global state variables
pose_t initial_pose = { 0.0f, 0.0f, 0.0f };
pose_t robot_pose = { 0.0f, 0.0f, 0.0f };
pose_t robot_vel = { 0.0f, 0.0f, 0.0f };
pose_t robot_odom = { 0.0f, 0.0f, 0.0f };
pose_t target_pose = { 0.0f, 0.0f, 0.0f };
bool have_target = false;
uint16_t robot_state = MANUAL;
uint8_t controlled_by = KEYBOARD;
uint8_t use_slam = false;
uint8_t use_amcl = false;
uint8_t use_gmapping = false;
uint8_t use_sim = false;
bool imu_initialized = false;
float initial_imu_yaw = 0.0f;
float last_safe_theta = 0.0f;
std::vector<point2d_t> lidar_data;

// Frame IDs - configurable frames
std::string map_frame_id = "map";
std::string odom_frame_id = "odom";
std::string base_frame_id = "base_footprint";
std::string laser_frame_id = "laser";
std::string imu_frame_id = "imu";

// Control inputs
axis_t axis_left = { 0.0f, 0.0f };
axis_t axis_right = { 0.0f, 0.0f };
button_t buttons = { 0, 0, 0, 0 };

// ROS objects
ros::Timer timer_main;
ros::Subscriber sub_joy;
ros::Subscriber sub_imu;
ros::Subscriber sub_lidar;
ros::Subscriber sub_encoder;
ros::Subscriber sub_amcl_pose;
ros::Subscriber sub_odom_filtered;
ros::Subscriber sub_initialpose;
ros::Subscriber sub_goal;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_robot_pose;
ros::Publisher pub_robot_odom;
ros::Publisher pub_raw_odom;
ros::Publisher pub_marker;
ros::Publisher pub_markers;
ros::Publisher pub_path;
ros::Publisher pub_imu;

tf::TransformBroadcaster* tf_broadcaster = nullptr;
tf::TransformListener* tf_listener = nullptr;

// Message storage
geometry_msgs::PoseWithCovarianceStamped amcl_pose;
sensor_msgs::Imu imu_msg;
nav_msgs::Path path_msg;

int8_t kbhit()
{
<<<<<<< HEAD
    static const int STDIN = 0;
    static bool initialized = false;
    static struct termios initial_settings, new_settings;

    if (!initialized) {
        tcgetattr(STDIN, &initial_settings);
        new_settings = initial_settings;
        new_settings.c_lflag &= ~ICANON;
        new_settings.c_lflag &= ~ECHO;
        new_settings.c_lflag &= ~ISIG;
        new_settings.c_cc[VMIN] = 0;
        new_settings.c_cc[VTIME] = 0;
        tcsetattr(STDIN, TCSANOW, &new_settings);
        initialized = true;
    }
=======
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);

    // Get parameters from the parameter server
    MAX_LIN_VEL = nh.param<float>("max_lin_vel", 0.5f);
    MAX_ANG_VEL = nh.param<float>("max_ang_vel", 0.2f);
    MAX_LIN_ACC = nh.param<float>("max_lin_acc", 0.2f);
    MAX_ANG_ACC = nh.param<float>("max_ang_acc", 0.1f);
    Kp = nh.param<float>("kp", 1.0f);
    Ki = nh.param<float>("ki", 0.0f);
    Kd = nh.param<float>("kd", 0.0f);
    Kp_angular = nh.param<float>("kp_ang", 1.0f);
    Ki_angular = nh.param<float>("ki_ang", 0.0f);
    Kd_angular = nh.param<float>("kd_ang", 0.0f);
    tf_lidar2base_x = nh.param<float>("tf_lidar2base_x", 0.0f);
    tf_lidar2base_y = nh.param<float>("tf_lidar2base_y", 0.0f);
    tf_lidar2base_theta = nh.param<float>("tf_lidar2base_theta", 0.0f);

    initial_pose.x = nh.param<float>("initial_pose_x", 0.0f);
    initial_pose.y = nh.param<float>("initial_pose_y", 0.0f);
    initial_pose.theta = nh.param<float>("initial_pose_theta", 0.0f);

    printf("======================================\n");
    printf("        ROBOT NODE PARAMETERS         \n");
    printf("======================================\n");
    printf("Initial Pose\t: (%.2f, %.2f, %.2f)\n", initial_pose.x, initial_pose.y, initial_pose.theta);
    printf("MAX_LIN_VEL\t: %.2f\n", MAX_LIN_VEL);
    printf("MAX_ANG_VEL\t: %.2f\n", MAX_ANG_VEL);
    printf("MAX_LIN_ACC\t: %.2f\n", MAX_LIN_ACC);
    printf("MAX_ANG_ACC\t: %.2f\n", MAX_ANG_ACC);
    printf("Kp\t\t: %.2f\n", Kp);
    printf("Ki\t\t: %.2f\n", Ki);
    printf("Kd\t\t: %.2f\n", Kd);
    printf("Kp_ang\t\t: %.2f\n", Kp_angular);
    printf("Ki_ang\t\t: %.2f\n", Ki_angular);
    printf("Kd_ang\t\t: %.2f\n", Kd_angular);
    printf("Lidar to Base\t: (%.2f, %.2f, %.2f)\n", tf_lidar2base_x, tf_lidar2base_y, tf_lidar2base_theta);
    printf("======================================\n");

    set_initial_pose(initial_pose.x, initial_pose.y, initial_pose.theta);

    sub_joy = nh.subscribe<sensor_msgs::Joy>("/device/joy", 1, joy_callback);
    sub_imu = nh.subscribe<sensor_msgs::Imu>("/device/imu/data", 1, imu_callback);
    sub_lidar = nh.subscribe<sensor_msgs::LaserScan>("/device/lidar/scan", 1, lidar_callback);
    sub_encoder = nh.subscribe<std_msgs::Int32MultiArray>("/device/motor/raw_enc", 1, encoder_callback);
    sub_amcl_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, amcl_pose_callback);
    sub_goal_pose = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goal_pose_callback);
    sub_init_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, init_pose_callback);
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
    pub_robot_pose = nh.advertise<geometry_msgs::Pose2D>("/robot/pose", 1);
    pub_robot_odom = nh.advertise<nav_msgs::Odometry>("/robot/odom", 1);
    pub_marker = nh.advertise<visualization_msgs::Marker>("/robot/marker", 1);
    timer_main = nh.createTimer(ros::Duration(0.01), timer_callback);

    tf_broadcaster = new tf::TransformBroadcaster;
    tf_listener = new tf::TransformListener;
>>>>>>> 8a04e197ce205f4bd173596cf633669370180b3a

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

void set_initial_pose(float x, float y, float theta)
{
<<<<<<< HEAD
    robot_pose.x = x;
    robot_pose.y = y;
    robot_pose.theta = theta;
=======
    if (use_sim)
        dummy_odom();

    state_control();
    publish_all();

    time_control = ros::Time::now();
}
>>>>>>> 8a04e197ce205f4bd173596cf633669370180b3a

    robot_vel.x = 0.0f;
    robot_vel.y = 0.0f;
    robot_vel.theta = 0.0f;

    robot_odom.x = 0.0f;
    robot_odom.y = 0.0f;
    robot_odom.theta = 0.0f;

    initial_imu_yaw = 0.0f;
    imu_initialized = false;
    last_safe_theta = theta;

    ROS_INFO("Robot pose reset to (%.2f, %.2f, %.2f)", x, y, theta);
}

void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    float x = msg->pose.pose.position.x * 100.0f; // Convert from meters to cm
    float y = msg->pose.pose.position.y * 100.0f; // Convert from meters to cm
    float theta = tf::getYaw(msg->pose.pose.orientation) * 180.0f / M_PI; // Convert from radians to degrees

    set_initial_pose(x, y, theta);
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
<<<<<<< HEAD
    target_pose.x = msg->pose.position.x * 100.0f; // Convert from meters to cm
    target_pose.y = msg->pose.position.y * 100.0f; // Convert from meters to cm
    target_pose.theta = tf::getYaw(msg->pose.orientation) * 180.0f / M_PI; // Convert from radians to degrees

    have_target = true;
    ROS_INFO("New goal received: (%.2f, %.2f, %.2f)", target_pose.x, target_pose.y, target_pose.theta);
=======
    static uint16_t state = 0;
    if (kbhit()) {
        char c = getchar();
        switch (c) {
        case 'w':
            state = 'w';
            break;
        case 's':
            state = 's';
            break;
        case 'a':
            state = 'a';
            break;
        case 'd':
            state = 'd';
            break;
        case 'q':
            state = 'q';
            break;
        case 'e':
            state = 'e';
            break;
        case ' ':
            state = ' ';
            break;
        case 'o':
            set_initial_pose(initial_pose.x, initial_pose.y, initial_pose.theta);
            break;
        case 'z':
            state = 'z';
            break;
        case 'c':
            state = 'c';
            break;
        case 'g':
            state = 'g';
            break;
        case 'x':
            controlled_by = JOYSTICK;
            ROS_INFO("Control mode: JOYSTICK");
            break;
        default:
            break;
        }
    }

    switch (state) {
    case 'w':
        velocity_control(0.0, MAX_LIN_VEL, 0.0);
        break;
    case 's':
        velocity_control(0.0, -MAX_LIN_VEL, 0.0);
        break;
    case 'a':
        velocity_control(-MAX_LIN_VEL, 0.0, 0.0);
        break;
    case 'd':
        velocity_control(MAX_LIN_VEL, 0.0, 0.0);
        break;
    case 'q':
        velocity_control(0.0, 0.0, MAX_ANG_VEL);
        break;
    case 'e':
        velocity_control(0.0, 0.0, -MAX_ANG_VEL);
        break;
    case ' ':
        velocity_control(0.0, 0.0, 0.0);
        break;
    case 'z':
        if (position_control(0.0, 1.0, 0.0)) {
            velocity_control(0.0, 0.0, 0.0);
            state = ' ';
        }
        break;
    case 'c':
        if (position_control(robot_pose.x, robot_pose.y, 0.0)) {
            velocity_control(0.0, 0.0, 0.0);
            state = ' ';
        }
        break;
    case 'g':
        if (position_control(goal_pose.x, goal_pose.y, goal_pose.theta)) {
            velocity_control(0.0, 0.0, 0.0);
            state = ' ';
        }
        break;
    default:
        break;
    }
>>>>>>> 8a04e197ce205f4bd173596cf633669370180b3a
}

void odom_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_pose.x = msg->pose.pose.position.x * 100.0f; // Convert to centimeters
    robot_pose.y = msg->pose.pose.position.y * 100.0f; // Convert to centimeters
    robot_pose.theta = tf::getYaw(msg->pose.pose.orientation) * 180.0f / M_PI;

    // Normalize angle to [-180, 180]
    if (robot_pose.theta > 180.0f) {
        robot_pose.theta -= 360.0f;
    } else if (robot_pose.theta < -180.0f) {
        robot_pose.theta += 360.0f;
    }
}

void publish_tf()
{
    ros::Time current_time = ros::Time::now();
    tf::Quaternion q;

    // Only publish map->odom if we're not using SLAM or GMAPPING
    if (!use_slam && !use_gmapping) {
        tf::Transform tf_map2odom;
        tf_map2odom.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        q.setRPY(0, 0, 0);
        tf_map2odom.setRotation(q);
        tf_broadcaster->sendTransform(tf::StampedTransform(tf_map2odom, current_time, map_frame_id, odom_frame_id));
    }

    // base_footprint -> base_link
    tf::Transform tf_base2link;
    tf_base2link.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setRPY(0, 0, 0);
    tf_base2link.setRotation(q);
    tf_broadcaster->sendTransform(tf::StampedTransform(tf_base2link, current_time, base_frame_id, "base_link"));

    // base_link -> laser
    tf::Transform tf_link2scan;
    tf_link2scan.setOrigin(tf::Vector3(tf_lidar2base_x, tf_lidar2base_y, tf_lidar2base_z));
    q.setRPY(tf_lidar2base_roll, tf_lidar2base_pitch, tf_lidar2base_yaw);
    tf_link2scan.setRotation(q);
    tf_broadcaster->sendTransform(tf::StampedTransform(tf_link2scan, current_time, "base_link", laser_frame_id));

    // base_link -> imu
    tf::Transform tf_base2imu;
    tf_base2imu.setOrigin(tf::Vector3(tf_imu2base_x, tf_imu2base_y, tf_imu2base_z));
    q.setRPY(tf_imu2base_roll, tf_imu2base_pitch, tf_imu2base_yaw);
    tf_base2imu.setRotation(q);
    tf_broadcaster->sendTransform(tf::StampedTransform(tf_base2imu, current_time, "base_link", imu_frame_id));
}

void publish_visualization()
{
<<<<<<< HEAD
    ros::Time current_time = ros::Time::now();
    visualization_msgs::Marker marker_msg;

    // Robot marker (cube)
    marker_msg.header.frame_id = odom_frame_id;
    marker_msg.header.stamp = current_time;
    marker_msg.ns = "robot";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::CUBE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.scale.x = 0.25;
    marker_msg.scale.y = 0.25;
    marker_msg.scale.z = 0.25;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;

    try {
        tf::StampedTransform transform;
        tf_listener->lookupTransform(odom_frame_id, base_frame_id, ros::Time(0), transform);
        marker_msg.pose.position.x = transform.getOrigin().x();
        marker_msg.pose.position.y = transform.getOrigin().y();
        marker_msg.pose.position.z = transform.getOrigin().z();
        marker_msg.pose.orientation.x = transform.getRotation().x();
        marker_msg.pose.orientation.y = transform.getRotation().y();
        marker_msg.pose.orientation.z = transform.getRotation().z();
        marker_msg.pose.orientation.w = transform.getRotation().w();
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Transform exception: %s", ex.what());
        return;
    }

    pub_marker.publish(marker_msg);
=======
    static float integral_x = 0.0, integral_y = 0.0, integral_theta = 0.0;
    static float prev_error_x = 0.0, prev_error_y = 0.0, prev_error_theta = 0.0;
    static double prev_time = ros::Time::now().toSec();
    static float prev_x = x;
    static float prev_y = y;
    static float prev_theta = theta;

    if (prev_x != x || prev_y != y) {
        integral_x = 0.0;
        integral_y = 0.0;
    }

    if (prev_theta != theta) {
        integral_theta = 0.0;
    }

    if (time_control - ros::Time::now() > ros::Duration(0.5)) {
        integral_x = 0.0;
        integral_y = 0.0;
        integral_theta = 0.0;
    }

    float error_x = x - robot_pose.x;
    float error_y = y - robot_pose.y;
    float error_theta = theta - robot_pose.theta;

    double current_time = ros::Time::now().toSec();
    double dt = current_time - prev_time;

    if (dt > 0.0) {
        integral_x += error_x * dt;
        integral_y += error_y * dt;
        integral_theta += error_theta * dt;

        float derivative_x = (error_x - prev_error_x) / dt;
        float derivative_y = (error_y - prev_error_y) / dt;
        float derivative_theta = (error_theta - prev_error_theta) / dt;

        float output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x;
        float output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y;
        float output_theta = (Kp_angular * error_theta + Ki_angular * integral_theta + Kd_angular * derivative_theta) * M_PI / 180.0;
>>>>>>> 8a04e197ce205f4bd173596cf633669370180b3a

    // Path visualization
    static tf::TransformListener listener;
    static bool first_path_point = true;

    if (first_path_point) {
        path_msg.header.frame_id = map_frame_id;
        first_path_point = false;
    }

<<<<<<< HEAD
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = current_time;
    pose_stamped.header.frame_id = base_frame_id;
    pose_stamped.pose.position.x = 0;
    pose_stamped.pose.position.y = 0;
    pose_stamped.pose.position.z = 0;
    pose_stamped.pose.orientation.w = 1.0;

    try {
        geometry_msgs::PoseStamped transformed_pose;
        listener.transformPose(map_frame_id, pose_stamped, transformed_pose);
        path_msg.header.stamp = current_time;
        path_msg.poses.push_back(transformed_pose);

        // Limit path size to prevent memory growth
        if (path_msg.poses.size() > 1000) {
            path_msg.poses.erase(path_msg.poses.begin());
=======
        if (fabs(error_x) < 0.10 && fabs(error_y) < 0.10 && fabs(error_theta) < 5) {
            return 1;
>>>>>>> 8a04e197ce205f4bd173596cf633669370180b3a
        }

        pub_path.publish(path_msg);
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Path transform exception: %s", ex.what());
    }
}

void publish_all()
{
    static uint8_t counter = 0;
    ros::Time current_time = ros::Time::now();

    // Command velocity
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.y = -robot_vel.x;
    cmd_vel.linear.x = robot_vel.y;
    cmd_vel.angular.z = robot_vel.theta;
    pub_cmd_vel.publish(cmd_vel);

    // Robot pose
    geometry_msgs::Pose2D pose_msg;
    pose_msg.x = robot_pose.x / 100.0; // Convert to meters
    pose_msg.y = robot_pose.y / 100.0; // Convert to meters
    pose_msg.theta = DEG2RAD(robot_pose.theta); // Convert to radians
    pub_robot_pose.publish(pose_msg);

    // IMU
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = imu_frame_id;
    pub_imu.publish(imu_msg);

    // Odometry
    nav_msgs::Odometry odom_msgs;
    odom_msgs.header.stamp = current_time;
    odom_msgs.header.frame_id = odom_frame_id;
    odom_msgs.child_frame_id = base_frame_id;

<<<<<<< HEAD
        // ========== base_footprint -> base_link ==========
        // tf::Transform tf_base2link;
        // tf_base2link.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        // q.setRPY(0, 0, 0);
        // tf_base2link.setRotation(q);
        // tf_broadcaster->sendTransform(tf::StampedTransform(tf_base2link, current_time, base_frame, "base_link"));

        // ========== base_link -> base_scan ==========
        tf::Transform tf_link2scan;
        tf_link2scan.setOrigin(tf::Vector3(tf_lidar2base_x, tf_lidar2base_y, 0));
        q.setRPY(0, 0, tf_lidar2base_theta);
        tf_link2scan.setRotation(q);
        tf_broadcaster->sendTransform(tf::StampedTransform(tf_link2scan, current_time, "base_link", scan_frame));

        visualization_msgs::Marker marker_msg;

        marker_msg.header.frame_id = odom_frame;
        marker_msg.header.stamp = current_time;
        marker_msg.ns = "robot";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::Marker::CUBE;
        marker_msg.action = visualization_msgs::Marker::ADD;
        marker_msg.scale.x = 0.25;
        marker_msg.scale.y = 0.25;
        marker_msg.scale.z = 0.25;
        marker_msg.color.a = 1.0;
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.0;

        try {
            tf::StampedTransform transform;
            tf_listener->lookupTransform("odom", base_frame, ros::Time(0), transform);
            marker_msg.pose.position.x = transform.getOrigin().x();
            marker_msg.pose.position.y = transform.getOrigin().y();
            marker_msg.pose.orientation.x = transform.getRotation().x();
            marker_msg.pose.orientation.y = transform.getRotation().y();
            marker_msg.pose.orientation.z = transform.getRotation().z();
            marker_msg.pose.orientation.w = transform.getRotation().w();
        } catch (tf::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
=======
    // Convert from cm to meters
    odom_msgs.pose.pose.position.x = robot_odom.x * 0.01;
    odom_msgs.pose.pose.position.y = robot_odom.y * 0.01;
    odom_msgs.pose.pose.position.z = 0.0;
>>>>>>> cbd22fdedd92d6f3749e3802b376c25bd23648dc

    odom_msgs.pose.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(robot_odom.theta));

    // Set velocities
    odom_msgs.twist.twist.linear.x = robot_vel.y * 0.01; // convert to m/s
    odom_msgs.twist.twist.linear.y = -robot_vel.x * 0.01; // convert to m/s
    odom_msgs.twist.twist.angular.z = DEG2RAD(robot_vel.theta);

    // Pose covariance
    odom_msgs.pose.covariance[0] = 0.01; // x position variance
    odom_msgs.pose.covariance[7] = 0.01; // y position variance
    odom_msgs.pose.covariance[35] = 0.1; // yaw variance

    // Twist covariance
    odom_msgs.twist.covariance[0] = 0.01; // x velocity variance
    odom_msgs.twist.covariance[7] = 0.01; // y velocity variance
    odom_msgs.twist.covariance[35] = 0.1; // angular velocity variance

    pub_robot_odom.publish(odom_msgs);

<<<<<<< HEAD
    // Publish TF and visualization at reduced rate to save CPU
    if (counter++ % 3 == 0) {
        publish_tf();
        publish_visualization();
=======
    float delta_imu = imu_yaw_deg - initial_imu_yaw;

    // Wrap to [-180, 180]
    if (delta_imu > 180.0)
        delta_imu -= 360.0;
    if (delta_imu < -180.0)
        delta_imu += 360.0;

    float new_theta = initial_pose_theta + delta_imu;

    // Wrap again
    if (new_theta > 180.0)
        new_theta -= 360.0;
    if (new_theta < -180.0)
        new_theta += 360.0;

    // ! SAFETY BECAUSE OF IMU NOISE !
    if (fabs(new_theta - last_safe_theta) < 20.0) {
        robot_pose.theta = new_theta;
        last_safe_theta = new_theta;
    } else {
        // Spike detected: ignore this update
        ROS_WARN_THROTTLE(1.0, "IMU spike detected: ignoring rotation jump");
>>>>>>> 8a04e197ce205f4bd173596cf633669370180b3a
    }
}

void dummy_odom()
{
    // Simple simulated odometry for testing/debugging
    robot_pose.x += robot_vel.y * 0.01 * cosf(DEG2RAD(robot_pose.theta)) - robot_vel.x * 0.01 * sinf(DEG2RAD(robot_pose.theta));
    robot_pose.y += robot_vel.y * 0.01 * sinf(DEG2RAD(robot_pose.theta)) + robot_vel.x * 0.01 * cosf(DEG2RAD(robot_pose.theta));
    robot_pose.theta += robot_vel.theta * 0.01;

    // Normalize angle to [-180, 180]
    if (robot_pose.theta > 180.0f) {
        robot_pose.theta -= 360.0f;
    } else if (robot_pose.theta < -180.0f) {
        robot_pose.theta += 360.0f;
    }
}

int main(int argc, char** argv)
{
<<<<<<< HEAD
    // Make odometry for giving linear x, y, and angular z based on omni 4 wheel robot encoder
    int32_t enc_buffer[4] = { -msg->data[0], -msg->data[1], -msg->data[2], -msg->data[3] };
    static int32_t enc_prev_buffer[4] = { enc_buffer[0], enc_buffer[1], enc_buffer[2], enc_buffer[3] };
    static int32_t enc_diff[4] = { 0, 0, 0, 0 };
    static double prev_time = ros::Time::now().toSec();

    double current_time = ros::Time::now().toSec();
    double dt = current_time - prev_time;

    static const float angle[4] = { 225, 315, 45, 135 };
    static const float ROBOT_RADIUS = 17.5; // Distance from the center to the wheels in centimeters

    for (int i = 0; i < 4; i++) {
        enc_diff[i] = enc_buffer[i] - enc_prev_buffer[i];
        enc_prev_buffer[i] = enc_buffer[i];

        if (enc_diff[i] > 32767) {
            enc_diff[i] -= 65536;
        } else if (enc_diff[i] < -32768) {
            enc_diff[i] += 65536;
        }
    }

    float dx = 0.0, dy = 0.0, dtheta = 0.0;
    for (int i = 0; i < 4; i++) {
        dx += (float)enc_diff[i] * cosf(angle[i] * M_PI / 180.0) * ENC2CM;
        dy += (float)enc_diff[i] * sinf(angle[i] * M_PI / 180.0) * ENC2CM;
    }

    dx = dx / 4.0;
    dy = dy / 4.0;
    dtheta = (float)(enc_diff[0] + enc_diff[1] + enc_diff[2] + enc_diff[3]) * ENC2DEG / (4.0 * ROBOT_RADIUS);

    robot_odom.x += dx;
    robot_odom.y += dy;
    robot_odom.theta += dtheta;

    printf("odom: %.2f %.2f %.2f\n", robot_odom.x, robot_odom.y, robot_odom.theta);

    nav_msgs::Odometry odom_msgs;
    odom_msgs.header.stamp = ros::Time::now();
    odom_msgs.header.frame_id = "odom";
    odom_msgs.child_frame_id = "base_link";

    odom_msgs.pose.pose.position.x = robot_odom.x * 0.01; // Convert to meters
    odom_msgs.pose.pose.position.y = robot_odom.y * 0.01; // Convert to meters
    odom_msgs.pose.pose.position.z = 0.0;

<<<<<<< HEAD
    tf::Quaternion q;
    q.setRPY(0, 0, DEG2RAD(robot_odom.theta));
    odom_msgs.pose.pose.orientation.x = q.x();
    odom_msgs.pose.pose.orientation.y = q.y();
    odom_msgs.pose.pose.orientation.z = q.z();
    odom_msgs.pose.pose.orientation.w = q.w();

    odom_msgs.twist.twist.linear.x = dx / dt * 0.01; // Convert to meters per second
    odom_msgs.twist.twist.linear.y = dy / dt * 0.01; // Convert to meters per second
    odom_msgs.twist.twist.angular.z = DEG2RAD(dtheta / dt);

    pub_robot_odom.publish(odom_msgs);
=======
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::MultiThreadedSpinner spinner(0);

    // Get parameters from parameter server
    private_nh.param<float>("max_lin_vel", MAX_LIN_VEL, 0.5f);
    private_nh.param<float>("max_ang_vel", MAX_ANG_VEL, 0.25f);
    private_nh.param<float>("max_lin_acc", MAX_LIN_ACC, 0.01f);
    private_nh.param<float>("max_ang_acc", MAX_ANG_ACC, 1.0f);
    private_nh.param<float>("kp", Kp, 0.1f);
    private_nh.param<float>("ki", Ki, 0.0f);
    private_nh.param<float>("kd", Kd, 0.0f);

    // Transform parameters
    private_nh.param<float>("tf_lidar2base_x", tf_lidar2base_x, 0.0f);
    private_nh.param<float>("tf_lidar2base_y", tf_lidar2base_y, 0.0f);
    private_nh.param<float>("tf_lidar2base_z", tf_lidar2base_z, 0.0f);
    private_nh.param<float>("tf_lidar2base_roll", tf_lidar2base_roll, 0.0f);
    private_nh.param<float>("tf_lidar2base_pitch", tf_lidar2base_pitch, 0.0f);
    private_nh.param<float>("tf_lidar2base_yaw", tf_lidar2base_yaw, 0.0f);

    private_nh.param<float>("tf_imu2base_x", tf_imu2base_x, 0.0f);
    private_nh.param<float>("tf_imu2base_y", tf_imu2base_y, 0.0f);
    private_nh.param<float>("tf_imu2base_z", tf_imu2base_z, 0.0f);
    private_nh.param<float>("tf_imu2base_roll", tf_imu2base_roll, 0.0f);
    private_nh.param<float>("tf_imu2base_pitch", tf_imu2base_pitch, 0.0f);
    private_nh.param<float>("tf_imu2base_yaw", tf_imu2base_yaw, 0.0f);

    // Initial pose
    private_nh.param<float>("initial_pose_x", initial_pose.x, 0.0f);
    private_nh.param<float>("initial_pose_y", initial_pose.y, 0.0f);
    private_nh.param<float>("initial_pose_theta", initial_pose.theta, 0.0f);

    // Mode settings
    private_nh.param<uint8_t>("use_amcl", use_amcl, 0);
    private_nh.param<uint8_t>("use_gmapping", use_gmapping, 0);
    private_nh.param<uint8_t>("use_slam", use_slam, 0);

    // Frame IDs
    private_nh.param<std::string>("map_frame_id", map_frame_id, "map");
    private_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    private_nh.param<std::string>("base_frame_id", base_frame_id, "base_link");
    private_nh.param<std::string>("laser_frame_id", laser_frame_id, "laser");
    private_nh.param<std::string>("imu_frame_id", imu_frame_id, "imu");

    // Print configuration
    ROS_INFO("======================================");
    ROS_INFO("        ROBOT NODE PARAMETERS         ");
    ROS_INFO("======================================");
    ROS_INFO("Initial Pose\t: (%.2f, %.2f, %.2f)", initial_pose.x, initial_pose.y, initial_pose.theta);
    ROS_INFO("MAX_LIN_VEL\t: %.2f", MAX_LIN_VEL);
    ROS_INFO("MAX_ANG_VEL\t: %.2f", MAX_ANG_VEL);
    ROS_INFO("MAX_LIN_ACC\t: %.2f", MAX_LIN_ACC);
    ROS_INFO("MAX_ANG_ACC\t: %.2f", MAX_ANG_ACC);
    ROS_INFO("PID\t\t: P=%.2f, I=%.2f, D=%.2f", Kp, Ki, Kd);
    ROS_INFO("Lidar to Base\t: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)",
        tf_lidar2base_x, tf_lidar2base_y, tf_lidar2base_z,
        tf_lidar2base_roll, tf_lidar2base_pitch, tf_lidar2base_yaw);
    ROS_INFO("IMU to Base\t: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)",
        tf_imu2base_x, tf_imu2base_y, tf_imu2base_z,
        tf_imu2base_roll, tf_imu2base_pitch, tf_imu2base_yaw);
    ROS_INFO("Use AMCL\t: %s", use_amcl ? "Yes" : "No");
    ROS_INFO("Use GMapping\t: %s", use_gmapping ? "Yes" : "No");
    ROS_INFO("Use SLAM\t: %s", use_slam ? "Yes" : "No");
    ROS_INFO("Frame IDs\t: map='%s', odom='%s', base='%s'",
        map_frame_id.c_str(), odom_frame_id.c_str(), base_frame_id.c_str());
    ROS_INFO("======================================");

    // Set initial pose
    set_initial_pose(initial_pose.x, initial_pose.y, initial_pose.theta);

    // Initialize TF objects
    tf_broadcaster = new tf::TransformBroadcaster();
    tf_listener = new tf::TransformListener();
>>>>>>> cbd22fdedd92d6f3749e3802b376c25bd23648dc

    // Initialize Path message
    path_msg.header.frame_id = map_frame_id;

    // Subscribe to topics
    sub_joy = nh.subscribe<sensor_msgs::Joy>("/device/joy", 1, joy_callback);
    sub_imu = nh.subscribe<sensor_msgs::Imu>("/device/imu/data", 1, imu_callback);
    sub_lidar = nh.subscribe<sensor_msgs::LaserScan>("/device/lidar/scan", 1, lidar_callback);
    sub_encoder = nh.subscribe<std_msgs::Int32MultiArray>("/device/motor/raw_enc", 1, encoder_callback);
    sub_amcl_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, amcl_pose_callback);
    sub_odom_filtered = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, odom_filtered_callback);
    sub_initialpose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, initialpose_callback);
    sub_goal = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goal_callback);

    // Publishers
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
    pub_robot_pose = nh.advertise<geometry_msgs::Pose2D>("/robot/pose", 1);
    pub_raw_odom = nh.advertise<nav_msgs::Odometry>("/robot/raw_odom", 1);
    pub_marker = nh.advertise<visualization_msgs::Marker>("/robot/marker", 1);
    pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/robot/markers", 1);
    pub_path = nh.advertise<nav_msgs::Path>("/robot/path", 1);
    pub_imu = nh.advertise<sensor_msgs::Imu>("/robot/imu", 1);

    // Main timer
    timer_main = nh.createTimer(ros::Duration(0.01), timer_callback);

    ROS_INFO("Robot node initialized and running.");
    spinner.spin();

    // Clean up
    delete tf_broadcaster;
    delete tf_listener;

    return 0;
=======
    update_robot_pose();
}

void goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_pose.x = msg->pose.position.x;
    goal_pose.y = msg->pose.position.y;
    goal_pose.theta = tf::getYaw(msg->pose.orientation) * 180 / M_PI;

    ROS_INFO("Received goal pose: x=%.2f, y=%.2f, theta=%.2f", goal_pose.x, goal_pose.y, goal_pose.theta);
}

void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    pose_t set_init;
    set_init.x = msg->pose.pose.position.x;
    set_init.y = msg->pose.pose.position.y;
    set_init.theta = tf::getYaw(msg->pose.pose.orientation) * 180 / M_PI;
    set_initial_pose(set_init.x, set_init.y, set_init.theta);

    ROS_INFO("Received init pose: x=%.2f, y=%.2f, theta=%.2f", set_init.x, set_init.y, set_init.theta);
>>>>>>> 8a04e197ce205f4bd173596cf633669370180b3a
}