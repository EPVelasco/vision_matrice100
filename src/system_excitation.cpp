#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class OdometryToVelocityNode {
public:
    OdometryToVelocityNode();

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher velocity_pub_;
    ros::Publisher odom_pub_;
    ros::Timer timer_;
    ros::Time start_time_;    
    nav_msgs::Odometry odometry;    
    double loop_rate = 0.05;

};

OdometryToVelocityNode::OdometryToVelocityNode() {
    // Obtener el parámetro de frecuencia del nodo
    nh_.param<double>("/loop_rate", loop_rate, 0.05);

    // Suscribirse al topic de odometría
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &OdometryToVelocityNode::odomCallback, this);

    // Publicar en el topic de velocidad
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Crear un temporizador con la frecuencia deseada
    timer_ = nh_.createTimer(ros::Duration(loop_rate), &OdometryToVelocityNode::timerCallback, this);

    // Inicializar el tiempo de inicio del nodo
    start_time_ = ros::Time::now();
}

void OdometryToVelocityNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Leer los datos de odometría del mensaje
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

void OdometryToVelocityNode::timerCallback(const ros::TimerEvent& event) {
    // Calcular el tiempo transcurrido desde el inicio del programa
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - start_time_;

    // Comprobar si ha pasado más de 30 segundos
    if (elapsed_time.toSec() >= 30.0) {
        ROS_INFO("El programa ha finalizado después de %f segundos.", elapsed_time.toSec());
        ros::shutdown(); // Detener el nodo
        return;
    }

    // Publicar la velocidad deseada y la odometria para nueva lectura
    geometry_msgs::Twist desired_velocity_;

    // Calcular la señal seno en función del tiempo transcurrido
    double t = elapsed_time.toSec();
    double v = sin(t);
    double t_x = cos(t);

    desired_velocity_.linear.z = v;  // fuerza        
    desired_velocity_.angular.x = t_x;    // torque x
    desired_velocity_.angular.y = t_x;    // torque y
    desired_velocity_.angular.z = t_x;    // torque z

    velocity_pub_.publish(desired_velocity_);
    odom_pub_.publish(odometry);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_to_velocity_node");
    OdometryToVelocityNode node;
    ros::spin();
    return 0;
}
