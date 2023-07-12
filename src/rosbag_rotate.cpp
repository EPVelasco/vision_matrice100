/*Este programa rota la nube de puntos que lee el sensor lidar livox en 180 grados */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class PointCloudProcessor
{
public:
    PointCloudProcessor() : nh_("~")
    {
        // Suscribirse al tópico de la nube de puntos original
        cloud_sub_ = nh_.subscribe("/livox/lidar_rotate", 1, &PointCloudProcessor::cloudCallback, this);

        // Publicar en el tópico de la nueva nube de puntos
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg)
    {
        // Convertir la nube de puntos de mensaje a un objeto PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_cloud_msg, *input_cloud);

        // Girar la nube de puntos 180 grados respecto al eje x
        pcl::PointCloud<pcl::PointXYZI>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*input_cloud, *rotated_cloud);
        for (auto& point : rotated_cloud->points)
        {
            point.x *= 1.0;
            point.y *= -1.0;
            point.z *= -1.0;
        }

        // Convertir la nube de puntos girada a un mensaje ROS
        sensor_msgs::PointCloud2 output_cloud_msg;
        pcl::toROSMsg(*rotated_cloud, output_cloud_msg);
        output_cloud_msg.header = input_cloud_msg->header;
        output_cloud_msg.header.frame_id = "livox_frame";  // Cambiar al nuevo frame_id deseado

        // Publicar la nueva nube de puntos
        cloud_pub_.publish(output_cloud_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_processor");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}