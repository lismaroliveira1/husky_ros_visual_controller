#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

class VelocityController
{
private:
    geometry_msgs::Twist msg;
    ros::Subscriber vel_sub;
    ros::Publisher vel_pub;
    ros::Timer timer_pub;
    double publish_interval;

public:
    VelocityController(ros::NodeHandle *nh) {
        msg.angular.x, msg.angular.y, msg.angular.z = 0,0,0;
        msg.linear.x, msg.linear.y, msg.linear.x = 0,0,0;
        publish_interval = 0.1;
        vel_sub = nh->subscribe("husky_velocity_controller/cmd_vel", 10, &VelocityController::velocityCallback, this);
        vel_pub = nh->advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
        timer_pub = nh->createTimer(ros::Duration(publish_interval), &VelocityController::timerCallback, this);

       
    }

    void velocityCallback(const geometry_msgs::Twist &msg) {
        ROS_INFO("Velocity: x: %f, y: %f", msg.linear.x, msg.linear.y);
    };
    void timerCallback(const ros::TimerEvent &event) {
        geometry_msgs::Twist msg_pub;
    }

};

int main (int argc, char **argv) {  
    ros::init(argc, argv, "filter_velocity");
    ros::NodeHandle nh;
    VelocityController velocitySubscribeer = VelocityController(&nh);
    ros::spin();

    return 0;
}

