#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

serial::Serial ser;

// Publisher for wheel pulses
ros::Publisher pulse_pub;

// Function to send velocity commands to the Arduino
void sendVelocityCommand(float vx, float vy, float omega)
{
    std::ostringstream command;
    command << vx << " " << vy << " " << omega << "\n";
    ser.write(command.str());
}

// Callback for the velocity topic
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float omega = msg->angular.z;

    sendVelocityCommand(vx, vy, omega);
}

// Timer callback to read pulses and publish them
void timerCallback(const ros::TimerEvent&)
{
    if (ser.available()) {
        try {
            // Read the response from Arduino
            std::string response = ser.readline();
            ROS_INFO_STREAM("Arduino Response: " << response);

            // Parse the response into individual wheel pulses
            std::istringstream iss(response);
            int wheel1, wheel2, wheel3, wheel4;
            iss >> wheel1 >> wheel2 >> wheel3 >> wheel4;

            // Publish the pulses as a ROS message
            std_msgs::Int32MultiArray pulse_msg;
            pulse_msg.data = { wheel1, wheel2, wheel3, wheel4 };
            pulse_pub.publish(pulse_msg);
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error reading from serial: " << e.what());
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_comm_node");
    ros::NodeHandle nh;

    // Subscriber for velocity commands
    ros::Subscriber velocity_sub = nh.subscribe("/robot/cmd_vel", 10, velocityCallback);

    // Publisher for wheel pulses
    pulse_pub = nh.advertise<std_msgs::Int32MultiArray>("/device/wheel_pulses", 10);

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

    // Create a timer to periodically read pulses and publish them
    ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback); // 50 Hz

    // Spin to process callbacks
    ros::spin();

    return 0;
}