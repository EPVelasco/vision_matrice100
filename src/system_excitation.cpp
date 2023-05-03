#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


class OdometryToVelocityNode {
public:
    OdometryToVelocityNode() {

        double loop_rate;
        nh.param("loop_rate", loop_rate, 0.05); 
        odom_sub = nh.subscribe("/odom", 1, &OdometryToVelocityNode::odomCallback, this);
        vel_pub  = nh.advertise<geometry_msgs::Twist>("/dron_Velocity", 1);
        odom_pub = nh.advertise<geometry_msgs::Twist>("/dron_Odometry", 1);
        loop_timer = nh.createTimer(ros::Duration(loop_rate), &OdometryToVelocityNode::timerCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
     

        double t = ros::Time::now().toSec();
        double v = sin(t);
        double t_x = cos(t);

        desired_velocity.linear.z = v;  // fuerza        
        desired_velocity.angular.x = t_x;    // torque x
        desired_velocity.angular.y = t_x;    // torque y
        desired_velocity.angular.z = t_x;    // torque z


        odometry.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        odometry.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        odometry.pose.pose.orientation.z = msg->pose.pose.orientation.z;
        odometry.pose.pose.orientation.w = msg->pose.pose.orientation.w;
        odometry.pose.pose.position.x = msg->pose.pose.position.x;
        odometry.pose.pose.position.y = msg->pose.pose.position.y;
        odometry.pose.pose.position.z = msg->pose.pose.position.z;
        odometry.twist.twist.linear.x = msg->twist.twist.linear.x;
        odometry.twist.twist.linear.y = msg->twist.twist.linear.x;
        odometry.twist.twist.linear.z = msg->twist.twist.linear.x;
        odometry.twist.twist.angular.x = msg->twist.twist.linear.x;
        odometry.twist.twist.angular.y = msg->twist.twist.linear.x;
        odometry.twist.twist.angular.z = msg->twist.twist.linear.x;


    }

    void timerCallback(const ros::TimerEvent& event) {
        vel_pub.publish(desired_velocity);
        odom_pub.publish(odometry);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher vel_pub;
    ros::Publisher odom_pub;
    ros::Timer loop_timer;
    geometry_msgs::Twist desired_velocity;
    nav_msgs::Odometry odometry;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_odom_node");
    std::cout<<"velocity and odom Node initialized... :)";
    OdometryToVelocityNode node;
    ros::spin();
    return 0;
}