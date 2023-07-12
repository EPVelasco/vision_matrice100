/*
Programa para hacer voxelizado a una nube de puntos, programa solo paa validacion de nubesde puntos
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>



typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class PointCloudProcessor
{
public:
    PointCloudProcessor() : nh_("~")
    {
        // Suscribirse al tópico de la nube de puntos original
        cloud_sub_ = nh_.subscribe("/velodyne_cloud_registered", 1, &PointCloudProcessor::cloudCallback, this);

        // Publicar en el tópico de la nueva nube de puntos
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/voxel_livox_mapping", 1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg)
    {

        // Convertir la nube de puntos de mensaje a un objeto PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_cloud_msg, *input_cloud);

        if (input_cloud == NULL) return;

        PointCloud::Ptr cloud_in (new PointCloud);
        PointCloud::Ptr cloud_out (new PointCloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*input_cloud, *cloud_in, indices);

        for (int i = 0; i < (int) cloud_in->points.size(); i++)
        {
            double distance = sqrt(cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y);
            if(distance<1.0 || distance>100.0)
                continue;
            cloud_out->push_back(cloud_in->points[i]);            
        }

        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud_out);
        vg.setLeafSize(0.5f, 0.5f, 0.1f);
        vg.filter(*cloud_out);
        
       
        // Convertir la nube de puntos a un mensaje ROS
        sensor_msgs::PointCloud2 output_cloud_msg;
        pcl::toROSMsg(*cloud_out, output_cloud_msg);
        output_cloud_msg.header = input_cloud_msg->header;
        output_cloud_msg.header.frame_id = "camera_init";  // Cambiar al nuevo frame_id deseado

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
    ros::init(argc, argv, "point_cloud_filtered");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}