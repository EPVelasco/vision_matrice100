#include <ros/ros.h>
#include <limits>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <chrono> 


// markers
#include <visualization_msgs/Marker.h>

#include "sensor_msgs/Joy.h"

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

typedef std::chrono::high_resolution_clock Clock;

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace nav_msgs;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

//Publisher
ros::Publisher livox_map_pub;          // path coordinates 

// topics a suscribirse del nodo
std::string pcl_topic = "/livox/lidar";
std::string odom_dron  = "/dji_sdk/odometry";
std::string tf_pcl     = "world";

bool tf_wOs = true;  // true mundo,  false sensor

float minlen = 0.5;
float maxlen = 200.0;
float angLin = 10.0;

int minCan = 80;
int maxCan = 100;

// matrices de calibracion entre la camara y el dron
Eigen::MatrixXf Tdc(3,1); // translation matrix dron-camera
Eigen::MatrixXf Rdc(3,3); // rotation matrix dron-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix


///////////////////////////////////////callback
void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& in_pc2, const OdometryConstPtr& odom_msg)
// void callback(const ImageConstPtr& in_RGB , const ImageConstPtr& in_depth)
{
      
  // Obtener la orientación del mensaje de odometría como un objeto Quaternion
  tf::Quaternion q;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, q);

  // Convertir el quaternion a una matriz de rotación
  tf::Matrix3x3 m(q);
  // Convertir la matriz de rotación a una matriz de transformación de la librería Eigen

  Eigen::Matrix4f  T_odom = Eigen::Matrix4f::Identity();
  
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T_odom(i, j) = m[i][j];
        }
    }
    T_odom(0, 3) = odom_msg->pose.pose.position.x;
    T_odom(1, 3) = odom_msg->pose.pose.position.y;
    T_odom(2, 3) = odom_msg->pose.pose.position.z;

//Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*in_pc2,pcl_pc2);
  PointCloud::Ptr msg_pointCloud(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*msg_pointCloud);
  ///

  ////// filter point cloud 
  if (msg_pointCloud == NULL) return;

  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg_pointCloud, *cloud_in, indices);



  for (int i = 0; i < (int) cloud_in->points.size(); i++)
  {
      double distance = sqrt(cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y);
      if(distance<minlen || distance>maxlen)
          continue;
     cloud_out->push_back(cloud_in->points[i]);
  }

   Eigen::MatrixXf RTdl(4,4); // transformation matrix dron-camera
  RTdl<<   1.0,  0.0 ,  0.0 ,  0.0
          ,0.0, -1.0 ,  0.0 ,  0.0
          ,0.0,  0.0 , -1.0 , -0.1
          ,0.0,  0.0 ,  0.0 ,  1.0 ;


  PointCloud::Ptr pcl_map (new PointCloud);
  PointCloud::Ptr point_var (new PointCloud);
  point_var->width = 1;
  point_var->height = 1;
  point_var->is_dense = false;
  point_var->points.resize (point_var->width * point_var->height);

  Eigen::MatrixXf pc_matrix(4,1);
  Eigen::MatrixXf points_lidar2dron(4,1);

 
  for (int i = 0; i < (int) cloud_out->points.size(); i++)
  {

        pc_matrix(0,0) = cloud_out->points[i].x ;
        pc_matrix(1,0) = cloud_out->points[i].y ;
        pc_matrix(2,0) = cloud_out->points[i].z ;
        pc_matrix(3,0) = 1.0;

        //T_odom(2, 3) = z_feat;

        points_lidar2dron = pc_matrix;
        points_lidar2dron = T_odom*(RTdl*pc_matrix);
  
        point_var->points[0].x = points_lidar2dron(0,0);
        point_var->points[0].y = points_lidar2dron(1,0);
        point_var->points[0].z = points_lidar2dron(2,0);
        point_var->points[0].intensity = cloud_out->points[i].intensity;
        pcl_map->push_back(point_var->points[0]);     
      
  }

  //std::cout<<"profundidad: "<<aver_depth<<std::endl;

  // features point cloud
  pcl_map->is_dense = false;
  pcl_map->width = pcl_map->width;
  pcl_map->height = pcl_map->height;
  pcl_map->points.resize (pcl_map->width * pcl_map->height);
  pcl_map->header.frame_id = tf_pcl;
  ros::Time time_st = odom_msg->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  pcl_map->header.stamp    =  time_st.toNSec()/1e3;
  livox_map_pub.publish(pcl_map);

}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "mapping_Drone");
  ros::NodeHandle nh;  
  
  /// Load Parameters
  std::cout<<"mapping Drone pcl Node initialized... :)";

  message_filters::Subscriber<PointCloud2>  pcl_livox(nh, pcl_topic, 5);
  message_filters::Subscriber<Odometry>     odom_robot(nh, odom_dron, 5);

  typedef sync_policies::ApproximateTime<PointCloud2, Odometry> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pcl_livox, odom_robot);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  livox_map_pub= nh.advertise<PointCloud> ("/livox/lidar/map", 10);
 
  ros::spin();
}