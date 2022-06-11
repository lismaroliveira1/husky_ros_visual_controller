#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class HuskyVelocityPublisher
{
private:
   geometry_msgs::Twist msg;
   double publish_interval;
   ros::Publisher vel_pub;
   ros::Timer timer_pub;


public:
    HuskyVelocityPublisher(ros::NodeHandle *nh) {
        msg.angular.x, msg.angular.y, msg.angular.z = 0,0,0;
        msg.linear.x, msg.linear.y, msg.linear.x = 0,0,0;
        publish_interval = 0.1;

        vel_pub = nh->advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
        timer_pub = nh->createTimer(ros::Duration(publish_interval), &HuskyVelocityPublisher::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent &event) {
        geometry_msgs::Twist msg_pub;
        msg.linear.x = msg.linear.x + 0.1;
        msg_pub.linear.x = msg.linear.x;
        vel_pub.publish(msg_pub);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_publisher");
    ros::NodeHandle nh;
    HuskyVelocityPublisher huskyVelocityPublisher = HuskyVelocityPublisher(&nh);
    ros::spin();
    return 0;
}
