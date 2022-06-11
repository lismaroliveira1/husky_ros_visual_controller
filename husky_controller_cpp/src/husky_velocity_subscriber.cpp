#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

class VelocitySubscriber
{
private:
    geometry_msgs::Twist msg;
    ros:: Subscriber vel_sub;
public:
    VelocitySubscriber(ros::NodeHandle *nh) {
        msg.angular.x, msg.angular.y, msg.angular.z = 0,0,0;
        msg.linear.x, msg.linear.y, msg.linear.x = 0,0,0;
        vel_sub = nh->subscribe("husky_velocity_controller/cmd_vel", 10, &VelocitySubscriber::velocityCallback, this);
       
    }

     void velocityCallback(const geometry_msgs::Twist &msg) {
        ROS_INFO("Velocity: x: %f, y: %f", msg.linear.x, msg.linear.y);
    };
};

int main (int argc, char **argv) {  
    ros::init(argc, argv, "filter_velocity");
    ros::NodeHandle nh;
    VelocitySubscriber velocitySubscribeer = VelocitySubscriber(&nh);
    ros::spin();

    return 0;
}

