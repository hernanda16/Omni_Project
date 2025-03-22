#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <serial/serial.h>
#include <sstream>

serial::Serial ser;

// Publisher for wheel pulses
ros::Publisher pulse_pub;

// Structure for wheel data
struct WheelData {
    int wheelULPulse;
    int wheelLLPulse;
    int wheelLRPulse;
    int wheelURPulse;
};

// Global variables to store velocity commands
float vx = 0.0;
float vy = 0.0;
float omega = 0.0;

// Function to send velocity commands to the Arduino
void sendVelocityCommand(float vx, float vy, float omega) {
    float buffer[3] = {vx, vy, omega};
    ser.write((uint8_t*)buffer, sizeof(buffer));
}

// Function to send PID parameters and pid_interval to the Arduino
void sendPIDParams(float kp, float ki, float kd, unsigned int pid_interval) {
    float pidParams[4] = {kp, ki, kd, static_cast<float>(pid_interval)};
    ser.write((uint8_t*)pidParams, sizeof(pidParams));
    ROS_INFO("Sent PID parameters: Kp=%.2f, Ki=%.2f, Kd=%.2f, pid_interval=%u", kp, ki, kd, pid_interval);
}

// Callback untuk menerima velocity
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    vx = msg->linear.x;
    vy = msg->linear.y;
    omega = msg->angular.z;

    printf("vx: %.2f, vy: %.2f, omega: %.2f\n", vx, vy, omega);
    sendVelocityCommand(vx, vy, omega);
}

// Timer callback to read pulses and publish them
void pulseTimerCallback(const ros::TimerEvent&) {
    if (ser.available() >= sizeof(WheelData)) {
        try {
            // Read wheel data from Arduino
            WheelData wheelData;
            ser.read((uint8_t*)&wheelData, sizeof(wheelData));

            // Publish the pulses as a ROS message
            std_msgs::Int32MultiArray pulse_msg;
            pulse_msg.data = {wheelData.wheelULPulse, wheelData.wheelLLPulse,
                              wheelData.wheelLRPulse, wheelData.wheelURPulse};
            pulse_pub.publish(pulse_msg);
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error reading from serial: " << e.what());
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_comm_node");
    ros::NodeHandle nh;

    // Subscriber for velocity commands
    ros::Subscriber velocity_sub = nh.subscribe("/robot/cmd_vel", 10, velocityCallback);

    // Publisher for wheel pulses
    pulse_pub = nh.advertise<std_msgs::Int32MultiArray>("/device/raw_enc", 10);

    // Initialize serial communication
    try {
        ser.setPort("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI05UAHH-if00-port0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open port");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO("Serial port initialized");
    } else {
        return -1;
    }

    // Get PID parameters and pid_interval from ROS parameter server
    float kp, ki, kd;
    int pid_interval;
    nh.param("motor_kp", kp, 1.5f);
    nh.param("motor_ki", ki, 0.05f);
    nh.param("motor_kd", kd, 0.0f);
    nh.param("pid_interval", pid_interval, 1);

    // Send PID parameters and interval to Arduino
    sendPIDParams(kp, ki, kd, static_cast<unsigned int>(pid_interval));

    // Create a timer to periodically read pulses and publish them
    ros::Timer pulse_timer = nh.createTimer(ros::Duration(0.02), pulseTimerCallback); // 50 Hz

    // Spin to process callbacks
    ros::spin();

    return 0;
}