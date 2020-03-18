#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/ColorRGBA.h>
#include <signal.h>

//  publish message
std_msgs::ColorRGBA msg;

//  publisher
ros::Publisher drive_pub;

//  callback for subscribe to "android/imu" topic
void androidCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
    msg.g = imu->linear_acceleration.z / 5;
    msg.r = imu->linear_acceleration.x / 5;
    //  publication message    
    drive_pub.publish(msg);
}

//  shutdown signal to stop the car
void mySigint(int sig)
{
    msg.g = 0;
    msg.r = 0;
    //  publication stop message
    drive_pub.publish(msg);
    //  ROS shutdown
    ros::shutdown();
}

int main(int argc, char** argv)
{
    //  node initialization
    ros::init(argc, argv, "android_controller");
    ros::NodeHandle node_handle;

    //  create publisher
    drive_pub = node_handle.advertise<std_msgs::ColorRGBA>("drive", 1000);
    //  create subscriber 
    ros::Subscriber android_sub = node_handle.subscribe("android/imu", 1000, androidCallback);

    //  setting new signal
    signal(SIGINT, mySigint);
    ros::spin();
}
