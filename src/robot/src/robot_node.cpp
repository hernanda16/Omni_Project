#include "robot/robot_node.hpp"

int main(int argc, char** argv)
{
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
    // sub_imu = nh.subscribe<sensor_msgs::Imu>("/device/imu/data", 1, imu_callback);
    sub_imu = nh.subscribe<std_msgs::Float32>("/device/imu/yaw", 1, imu_callback);
    sub_lidar = nh.subscribe<sensor_msgs::LaserScan>("/device/lidar/scan", 1, lidar_callback);
    sub_encoder = nh.subscribe<std_msgs::Int32MultiArray>("/device/motor/raw_enc", 1, encoder_callback);
    sub_amcl_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, amcl_pose_callback);
    sub_goal_pose = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goal_pose_callback);
    sub_init_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, init_pose_callback);
    sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_vel_callback);
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
    pub_robot_pose = nh.advertise<geometry_msgs::Pose2D>("/robot/pose", 1);
    pub_robot_odom = nh.advertise<nav_msgs::Odometry>("/robot/odom", 1);
    pub_marker = nh.advertise<visualization_msgs::Marker>("/robot/marker", 1);
    timer_main = nh.createTimer(ros::Duration(0.01), timer_callback);

    tf_broadcaster = new tf::TransformBroadcaster;
    tf_listener = new tf::TransformListener;

    spinner.spin();
    return 0;
}

void timer_callback(const ros::TimerEvent&)
{
    if (use_sim)
        dummy_odom();

    keyboard_input();

    state_control();

    if (!use_dwa) {
        dwa_vel.x = 0;
        dwa_vel.y = 0;
        dwa_vel.theta = 0;
    }

    publish_all();

    time_control = ros::Time::now();
}

void dummy_odom()
{
    robot_pose.x += robot_vel.y * 0.01 * cosf(DEG2RAD(robot_pose.theta));
    robot_pose.y += robot_vel.y * 0.01 * sinf(DEG2RAD(robot_pose.theta));
}

void update_robot_pose()
{
    float weight_amcl = 0.1;
    float weight_odom = 1.0 - weight_amcl;

    robot_pose.x = weight_amcl * amcl_pose.pose.pose.position.x + weight_odom * (robot_pose.x + robot_vel.y * 0.01 * cosf(DEG2RAD(robot_pose.theta)));
    robot_pose.y = weight_amcl * amcl_pose.pose.pose.position.y + weight_odom * (robot_pose.y + robot_vel.y * 0.01 * sinf(DEG2RAD(robot_pose.theta)));
    robot_pose.theta = weight_amcl * (tf::getYaw(amcl_pose.pose.pose.orientation) * 180 / M_PI) + weight_odom * robot_pose.theta;

    if (robot_pose.theta > 180.0) {
        robot_pose.theta -= 360.0;
    } else if (robot_pose.theta < -180.0) {
        robot_pose.theta += 360.0;
    }
}

float compute_amcl_trust()
{
    double position_uncertainty = sqrt(amcl_pose.pose.covariance[0] + amcl_pose.pose.covariance[7]);
    double orientation_uncertainty = sqrt(amcl_pose.pose.covariance[35]);

    float trust = exp(-position_uncertainty - orientation_uncertainty);
    return std::max(0.1f, std::min(0.9f, trust)); // Keep trust between 0.1 and 0.9
}

void keyboard_input()
{
    if (kbhit()) {
        char c = getchar();
        switch (c) {
        case 'w':
            keyboard_state = 'w';
            break;
        case 's':
            keyboard_state = 's';
            break;
        case 'a':
            keyboard_state = 'a';
            break;
        case 'd':
            keyboard_state = 'd';
            break;
        case 'q':
            keyboard_state = 'q';
            break;
        case 'e':
            keyboard_state = 'e';
            break;
        case ' ':
            keyboard_state = ' ';
            use_dwa = 0;
            break;
        case 'o':
            set_initial_pose(initial_pose.x, initial_pose.y, initial_pose.theta);
            break;
        case 'z':
            keyboard_state = 'z';
            break;
        case 'c':
            keyboard_state = 'c';
            break;
        case 'g':
            keyboard_state = 'g';
            break;
        case 'x':
            controlled_by = !controlled_by;
            if (controlled_by)
                ROS_INFO("Control mode: JOYSTICK");
            else
                ROS_INFO("Control mode: KEYBOARD");
            break;
        case 'v':
            use_dwa = 1;
            ROS_INFO("USE DWA %d", use_dwa);
            break;
        default:
            break;
        }
    }
}
void keyboard_handler()
{
    switch (keyboard_state) {
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
            keyboard_state = 1010;
        }
        break;
    case 1010:
        if (position_control(-1.0, 1.0, 0.0)) {
            keyboard_state = 1011;
        }
        break;
    case 1011:
        if (position_control(-1.0, 0.0, 0.0)) {
            keyboard_state = 1012;
        }
        break;
    case 1012:
        if (position_control(0.0, 0.0, 0.0)) {
            keyboard_state = 'z';
        }
        break;
    case 'c':
        if (position_control(robot_pose.x, robot_pose.y, 0.0)) {
            velocity_control(0.0, 0.0, 0.0);
            keyboard_state = ' ';
        }
        break;
    case 'g':
        if (position_control(goal_pose.x, goal_pose.y, goal_pose.theta)) {
            velocity_control(0.0, 0.0, 0.0);
            keyboard_state = ' ';
        }
        break;
    default:
        break;
    }
}

void joystick_handler()
{
    velocity_control(axis_left.x * MAX_LIN_VEL, axis_left.y * MAX_LIN_VEL, axis_right.x * MAX_ANG_VEL);
}

void state_control()
{
    if (controlled_by == KEYBOARD)
        keyboard_handler();
    else if (controlled_by == JOYSTICK)
        joystick_handler();
}

void velocity_control(float vx, float vy, float vtheta)
{
    printf("vx: %.2f, vy: %.2f, vtheta: %.2f\n", vx, vy, vtheta);
    static pose_t prev_robot_vel = { 0.0, 0.0, 0.0 };
    static double prev_time = ros::Time::now().toSec();
    double dt = ros::Time::now().toSec() - prev_time;

    if (vx > prev_robot_vel.x + MAX_LIN_ACC * dt) {
        robot_vel.x = prev_robot_vel.x + MAX_LIN_ACC * dt;
    } else if (vx < prev_robot_vel.x - MAX_LIN_ACC * dt) {
        robot_vel.x = prev_robot_vel.x - MAX_LIN_ACC * dt;
    }

    if (vy > prev_robot_vel.y + MAX_LIN_ACC * dt) {
        robot_vel.y = prev_robot_vel.y + MAX_LIN_ACC * dt;
    } else if (vy < prev_robot_vel.y - MAX_LIN_ACC * dt) {
        robot_vel.y = prev_robot_vel.y - MAX_LIN_ACC * dt;
    }

    if (vtheta > prev_robot_vel.theta + MAX_ANG_ACC * dt) {
        robot_vel.theta = prev_robot_vel.theta + MAX_ANG_ACC * dt;
    } else if (vtheta < prev_robot_vel.theta - MAX_ANG_ACC * dt) {
        robot_vel.theta = prev_robot_vel.theta - MAX_ANG_ACC * dt;
    }

    prev_time = ros::Time::now().toSec();
    prev_robot_vel = robot_vel;
}

uint8_t position_control(float x, float y, float theta)
{
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

        velocity_control(-output_y, output_x, output_theta);

        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_theta = error_theta;
        prev_time = current_time;

        if (fabs(error_x) < 0.10 && fabs(error_y) < 0.10 && fabs(error_theta) < 5) {
            return 1;
        }
    }
    return 0;
}

void publish_all()
{
    static uint8_t counter = 0;
    ros::Time current_time = ros::Time::now();

    if (counter++ % 3 == 0) {
        geometry_msgs::Twist cmd_vel;
        if (use_dwa) {
            cmd_vel.linear.y = dwa_vel.y;
            cmd_vel.linear.x = dwa_vel.x;
            cmd_vel.angular.z = dwa_vel.theta;
        } else {
            cmd_vel.linear.y = robot_vel.y;
            cmd_vel.linear.x = robot_vel.x;
            cmd_vel.angular.z = robot_vel.theta;
        }
        pub_cmd_vel.publish(cmd_vel);

        geometry_msgs::Pose2D pose_msg;
        pose_msg.x = robot_pose.x;
        pose_msg.y = robot_pose.y;
        pose_msg.theta = robot_pose.theta;
        pub_robot_pose.publish(pose_msg);

        std::string map_frame = "map";
        std::string odom_frame = use_sim ? "odom_fake" : "odom";
        std::string base_frame = use_sim ? "base_footprint_fake" : "base_footprint";
        std::string scan_frame = use_sim ? "base_scan" : "laser";
        tf::Quaternion q;

        // ========== map -> odom ==========
        if (use_slam || use_gmapping) {
            // Let AMCL or GMapping publish this!
            // Do NOT publish it here!
        } else {
            tf::Transform tf_map2odom;
            tf_map2odom.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            tf_map2odom.setRotation(q);
            tf_broadcaster->sendTransform(tf::StampedTransform(tf_map2odom, current_time, map_frame, odom_frame));
        }

        // ========== Publish /robot/odom ==========
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = odom_frame; // "odom" or "odom_fake"
        odom_msg.child_frame_id = base_frame; // "base_footprint" or "base_footprint_fake"

        // Position
        odom_msg.pose.pose.position.x = robot_pose.x;
        odom_msg.pose.pose.position.y = robot_pose.y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(robot_pose.theta));

        // Velocity
        odom_msg.twist.twist.linear.x = robot_vel.x;
        odom_msg.twist.twist.linear.y = robot_vel.y;
        odom_msg.twist.twist.angular.z = robot_vel.theta;

        pub_robot_odom.publish(odom_msg); // <<< make sure you have a publisher ready

        // ========== odom -> base_footprint ==========
        tf::Transform tf_odom2base;
        tf_odom2base.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
        tf_odom2base.setRotation(tf::createQuaternionFromYaw(DEG2RAD(robot_pose.theta)));
        tf_broadcaster->sendTransform(tf::StampedTransform(tf_odom2base, current_time, odom_frame, base_frame));

        // ========== base_footprint -> base_link ==========
        tf::Transform tf_base2link;
        tf_base2link.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        q.setRPY(0, 0, 0);
        tf_base2link.setRotation(q);
        tf_broadcaster->sendTransform(tf::StampedTransform(tf_base2link, current_time, base_frame, "base_link"));

        // ========== base_link -> base_scan ==========
        tf::Transform tf_link2scan;
        tf_link2scan.setOrigin(tf::Vector3(tf_lidar2base_x, tf_lidar2base_y, 0));
        q.setRPY(0, 0, DEG2RAD(tf_lidar2base_theta));
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

        pub_marker.publish(marker_msg);
    }
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    axis_left.x = -msg->axes[0];
    axis_left.y = msg->axes[1];
    axis_right.x = msg->axes[2];
    axis_right.y = -msg->axes[3];

    buttons.x = msg->buttons[0];
    buttons.o = msg->buttons[1];
    buttons.square = msg->buttons[2];
    buttons.triangle = msg->buttons[3];

    if (buttons.x == 1) {
        controlled_by = !controlled_by;
        if (controlled_by)
            ROS_INFO("Control mode: JOYSTICK");
        else
            ROS_INFO("Control mode: KEYBOARD");
    }

    if (buttons.o == 1) {
        set_initial_pose(initial_pose.x, initial_pose.y, initial_pose.theta);
    }

    if (buttons.triangle == 1) {
        use_dwa = 1;
    }

    // printf("left: (x: %.2f, y: %.2f), right: (x: %.2f, y: %.2f)\n", axis_left.x, axis_left.y, axis_right.x, axis_right.y);
    // printf("buttons: (x: %d, o: %d, sq: %d, tr: %d)\n", buttons.x, buttons.o, buttons.square, buttons.triangle);
}

void imu_callback(const std_msgs::Float32::ConstPtr& msg)
{
    // Extract yaw from quaternion (in degrees)
    float imu_yaw_deg = -msg->data;

    if (!imu_initialized) {
        initial_imu_yaw = imu_yaw_deg;
        imu_initialized = true;
        return;
    }

    // Compute the offset between the initial IMU yaw and the current IMU yaw
    float imu_offset_yaw = imu_yaw_deg - initial_imu_yaw;

    // Wrap imu_offset_yaw into [-180, 180]
    if (imu_offset_yaw > 180.0f)
        imu_offset_yaw -= 360.0f;
    if (imu_offset_yaw < -180.0f)
        imu_offset_yaw += 360.0f;

    // Update robot's theta using the IMU offset
    float new_theta = initial_pose.theta + imu_offset_yaw + initial_pose_theta;

    // Wrap new_theta into [-180, 180]
    if (new_theta > 180.0f)
        new_theta -= 360.0f;
    if (new_theta < -180.0f)
        new_theta += 360.0f;

    robot_pose.theta = new_theta;
    // printf("Theta: %.2f\n", robot_pose.theta);
}
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    point2d_t temp;
    lidar_data.clear();

    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < msg->range_max) {
            float angle = msg->angle_min + i * msg->angle_increment + tf_lidar2base_theta;
            temp.x = msg->ranges[i] * cos(angle);
            temp.y = msg->ranges[i] * sin(angle);
            lidar_data.push_back(temp);
        }
    }

    for (int i = 0; i < lidar_data.size(); i++) {
        float x = lidar_data[i].x;
        float y = lidar_data[i].y;
        // Apply the transformation from lidar to base
        float x_base = x * cos(tf_lidar2base_theta) - y * sin(tf_lidar2base_theta) + tf_lidar2base_x;
        float y_base = x * sin(tf_lidar2base_theta) + y * cos(tf_lidar2base_theta) + tf_lidar2base_y;
        // Apply the transformation from base to world
        lidar_data[i].x = (x_base * cos(robot_pose.theta) - y_base * sin(robot_pose.theta) + robot_pose.x) * 0.001;
        lidar_data[i].y = (x_base * sin(robot_pose.theta) + y_base * cos(robot_pose.theta) + robot_pose.y) * 0.001;
    }
}

void encoder_callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    static const float angle[4] = { 45, 135, 225, 315 };
    int32_t enc_buffer[4] = { msg->data[0], msg->data[1], msg->data[2], msg->data[3] };
    static int32_t enc_prev_buffer[4] = { enc_buffer[0], enc_buffer[1], enc_buffer[2], enc_buffer[3] };
    static int32_t enc_diff[4] = { 0, 0, 0, 0 };
    static double prev_time = ros::Time::now().toSec();

    double current_time = ros::Time::now().toSec();
    double dt = current_time - prev_time;

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
        dx += (float)enc_diff[i] * 0.01 * cosf(angle[i] * M_PI / 180.0) * ENC2CM;
        dy += (float)enc_diff[i] * 0.01 * sinf(angle[i] * M_PI / 180.0) * ENC2CM;
    }

    robot_pose.y += -(dx * cosf(robot_pose.theta * M_PI / 180.0) - dy * sinf(robot_pose.theta * M_PI / 180.0)) * dt;
    robot_pose.x += (dx * sinf(robot_pose.theta * M_PI / 180.0) + dy * cosf(robot_pose.theta * M_PI / 180.0)) * dt;

    prev_time = current_time;
}

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    amcl_pose = *msg;

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
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Rotate the velocity by 90 degrees
    float rotated_x = -msg->linear.y;
    float rotated_y = msg->linear.x;

    dwa_vel.x = rotated_x * 1;
    dwa_vel.y = rotated_y * 1;
    dwa_vel.theta = msg->angular.z * 1;
}