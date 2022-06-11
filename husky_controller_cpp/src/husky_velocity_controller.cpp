#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

class VelocityController
{
private:
    geometry_msgs::Twist msg;
    double husky_pose_x;
    double husky_pose_y;
    double husky_orientation_theta;
    nav_msgs::Odometry goal_pose;
    ros::Subscriber vel_sub;
    ros::Subscriber pose_subscriber;
    ros::Publisher vel_pub;
    ros::Timer timer_pub;
    double publish_interval;

public:
    VelocityController(ros::NodeHandle *nh)
    {
        msg.angular.x, msg.angular.y, msg.angular.z = 0, 0, 0;
        msg.linear.x, msg.linear.y, msg.linear.x = 0, 0, 0;
        publish_interval = 0.1;
        vel_sub = nh->subscribe("husky_velocity_controller/cmd_vel", 10, &VelocityController::velocityCallback, this);
        pose_subscriber = nh->subscribe("husky_velocity_controller/odom", 10, &VelocityController::poseCallback, this);
        vel_pub = nh->advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
        goal_pose.pose.pose.orientation.x = 15;
        goal_pose.pose.pose.orientation.y = 29;

        goToGoal(goal_pose, 0.5);
        //timer_pub = nh->createTimer(ros::Duration(publish_interval), &VelocityController::timerCallback, this);
    }

    void velocityCallback(const geometry_msgs::Twist &msg)
    {
        ROS_INFO("Velocity: x: %f, y: %f", msg.linear.x, msg.linear.y);
    };
    void poseCallback(const nav_msgs::Odometry &pose)
    {
        husky_pose_x = pose.pose.pose.position.x;
        husky_pose_y = pose.pose.pose.position.y;
        husky_orientation_theta = acos(husky_pose_y/husky_pose_x);
    };
    void timerCallback(const ros::TimerEvent &event)
    {
        geometry_msgs::Twist msg_pub;
    }
    double getDistance(double desired_pose_x, double desired_pose_y)
    {
        return sqrt(pow(desired_pose_x - husky_pose_x, 2) - pow(desired_pose_y - husky_pose_y, 2));
    }

    void goToGoal(nav_msgs::Odometry goal_pose, double distante_tolerance)
    {
        geometry_msgs::Twist vel_msg;
        ros::Rate loop_rate(10);
        double desired_pose_x = goal_pose.pose.pose.position.x;
        double desired_pose_y = goal_pose.pose.pose.position.y;
        double Kpl = 1.2; //proportional gain to linear velocity
        double kpa = 0.05; //proportional gain to angular velocity
        do
        {
            double error = getDistance(desired_pose_x, desired_pose_y);
            vel_msg.linear.x = Kpl*error;
            vel_msg.angular.z = 4*(atan2(desired_pose_y - husky_pose_y, desired_pose_x - husky_pose_x) - husky_orientation_theta);
            vel_pub.publish(vel_msg);
            
        } while (getDistance(desired_pose_x, desired_pose_y) < distante_tolerance);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_velocity");
    ros::NodeHandle nh;
    VelocityController velocitySubscribeer = VelocityController(&nh);
    ros::spin();

    return 0;
}
