#include <ros/ros.h>
#include <limits>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/NavSatFix.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <chrono> 

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

typedef std::chrono::high_resolution_clock Clock;

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace nav_msgs;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

//Publisher
ros::Publisher featuresRGB_pub;   // features in RGB image
ros::Publisher featuresDepth_pub; // features in depth image
ros::Publisher featurespcl_pub;   // features in pcl
ros::Publisher path_pub;          // path coordinates 
ros::Publisher vis_pub;           // markers


//msgs

// topics a suscribirse del nodo
std::string imgTopic =   "/Matrice/camera/rgb/image_color";
std::string depthTopic = "/Matrice/camera/depth/image_raw";
std::string odom_dron = "/webots/odometry";
std::string tf_pcl = "camera_link";
bool tf_wOs = true;  // true mundo,  false sensor

bool real_flag = 0;
float minlen = 0.5;
float maxlen = 10.0;
float angLin = 10.0;

// matrices de calibracion entre la camara y el dron
Eigen::MatrixXf Tdc(3,1); // translation matrix dron-camera
Eigen::MatrixXf Rdc(3,3); // rotation matrix dron-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix


///////////////////////////////////////callback
void callback(const ImageConstPtr& in_RGB , const boost::shared_ptr<const sensor_msgs::PointCloud2>& in_pc2, const OdometryConstPtr& odom_msg)
{

  auto t1 = Clock::now();

  //////////////7////////// mensaje de odometria a matriz homogenea //////////////////7

  // Acceder a los datos del mensaje de Odometry

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

  ///////////////////////////////////////////////////////////////////////////////////////////

  /////////////////////// msgs image to opencvIamge 


    cv_bridge::CvImagePtr cv_rgbCam;
        try
        {
          cv_rgbCam      = cv_bridge::toCvCopy(in_RGB, sensor_msgs::image_encodings::BGR8);         // imagen de color de la camara 
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

  cv::Mat img_rgbCam    = cv_rgbCam->image;
  
  int cols_img = in_RGB->width;
  int rows_img = in_RGB->height;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////// msg point cloud to PCL library /////////////////////////////////

  //Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*in_pc2,pcl_pc);
  PointCloud::Ptr msg_pointCloud(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc,*msg_pointCloud);
  ///
  ////// filter point cloud 
  if (msg_pointCloud == NULL) return;
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg_pointCloud, *cloud_in, indices);

  float max_z = 0, min_z = std::numeric_limits<float>::infinity();
  float max_dis = 0, min_dis = std::numeric_limits<float>::infinity();
  
  for (int i = 0; i < (int) cloud_in->points.size(); i++)
  {
      double distance = sqrt(cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y);     
      if(distance<minlen || distance>maxlen)
       continue;

      cloud_out->push_back(cloud_in->points[i]);     
      if(cloud_in->points[i].z >max_z)
        max_z = cloud_in->points[i].z;
      if(cloud_in->points[i].z <min_z)
        min_z = cloud_in->points[i].z;
      if(distance>max_dis)
        max_dis = distance;
      if(distance<min_dis)
        min_dis = distance;      
    
  }  

  ///////////////////////////////////////////////////////////////////////////////////////////////////////


  ////////////////////////////////////////////// LiDAR CAmera Fusion///////////////////////////////////

  Eigen::MatrixXf RTlc(4,4); // translation matrix lidar-camera
  RTlc<<   Rlc(0), Rlc(1) , Rlc(2) ,Tlc(0)
          ,Rlc(3), Rlc(4) , Rlc(5) ,Tlc(1)
          ,Rlc(6), Rlc(7) , Rlc(8) ,Tlc(2)
          ,0       , 0        , 0  , 1    ;

  int size_inter_Lidar = (int) cloud_out->points.size();

  Eigen::MatrixXf Lidar_camera(3,size_inter_Lidar);
  Eigen::MatrixXf Lidar_cam(3,1);
  Eigen::MatrixXf Lidar_cam_depth(3,1); // convierto las profundidas del eje del lidar hacia la posicion de la camara
  Eigen::MatrixXf pc_matrix(4,1);
  Eigen::MatrixXf pointCloud_matrix(4,size_inter_Lidar);

  uint px_data = 0; uint py_data = 0;

  pcl::PointXYZRGB point;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_color (new pcl::PointCloud<pcl::PointXYZRGB>);
  cv::Mat depthImage =  cv::Mat::zeros(rows_img, cols_img, cv_bridge::getCvType("mono16"));

  cv::Mat color_pcImg = img_rgbCam.clone(); 

  for (int i = 0; i < size_inter_Lidar; i++)
  {
      pc_matrix(0,0) =  cloud_out->points[i].x;
      pc_matrix(1,0) =  cloud_out->points[i].y;
      pc_matrix(2,0) =  cloud_out->points[i].z;
      pc_matrix(3,0) = 1.0;

      Lidar_cam =  Mc * (RTlc * pc_matrix);
      Lidar_cam_depth = (RTlc * pc_matrix);


      px_data = (int)(Lidar_cam(0,0)/Lidar_cam(2,0));
      py_data = (int)(Lidar_cam(1,0)/Lidar_cam(2,0));
      
      if(px_data<0.0 || px_data>=cols_img || py_data<0.0 || py_data>=rows_img)
          continue;

      //int color_dis_x = (int)255 * ((sqrt(P_out->points[i].x * P_out->points[i].x + P_out->points[i].y * P_out->points[i].y)-min_dis)/(max_dis-min_dis));
      // coloreado de punot sobre la imagen RGB con respecto al lidar como origen 
      //int color_dis_x = (int)255*((P_out->points[i].x - min_dis)/(max_dis-min_dis));
      //int color_dis_z = (int)255*((P_out->points[i].x - min_z)/(max_z-min_z));
      // coloreado de punot sobre la imagen RGB con respecto a la camara como origen 

      int color_dis_x = (int)255*((Lidar_cam_depth(0,0) - min_dis)/(max_dis-min_dis));
      int color_dis_z = (int)255*((Lidar_cam_depth(0,0) - min_z)/(max_z-min_z));

      if(color_dis_z>255)
          color_dis_z = 255;

      //point cloud con color
      cv::Vec3b & color = color_pcImg.at<cv::Vec3b>(py_data,px_data);

      point.x = P_out->points[i].x;
      point.y = P_out->points[i].y;
      point.z = P_out->points[i].z;
      point.r = (int)color[2]; 
      point.g = (int)color[1]; 
      point.b = (int)color[0];
      
      pc_color->points.push_back(point);   

      //colored image
      cv::circle(cv_rgbCam->image, cv::Point(px_data, py_data), 1, CV_RGB(255-color_dis_x,(int)(color_dis_z),color_dis_x),cv::FILLED);

      //depth image 
      //int depth_range = (int)pow(2,16) * ((sqrt(P_out->points[i].x * P_out->points[i].x + P_out->points[i].y * P_out->points[i].y + P_out->points[i].z * P_out->points[i].z))/(200.0));//convertion range meters to 2**16
      //int depth_range = (int)pow(2,16) * ((sqrt(Lidar_cam_depth(0,0) * Lidar_cam_depth(0,0) + Lidar_cam_depth(1,0) * Lidar_cam_depth(1,0) + Lidar_cam_depth(2,0) * Lidar_cam_depth(2,0)))/(200.0));//convertion range meters to 2**16
      //int depth_range = (int)pow(2,16) * ((sqrt(Lidar_cam_depth(2,0) * Lidar_cam_depth(2,0))-0.30)/(200.0));//convertion range meters to 2**16

      int depth_range = (int)pow(2,16) * ((Lidar_cam_depth(2,0)-min_dis)/(max_dis-min_dis));//convertion range meters to 2**16

      depthImage.at<ushort>(py_data,px_data) = depth_range;  

      
  }

    // features point cloud
  pc_color->is_dense = false;
  pc_color->width = pc_color->width;
  pc_color->height = pc_color->height;
  pc_color->points.resize (pc_color->width * pc_color->height);
  pc_color->header.frame_id = tf_pcl;
  ros::Time time_st = in_depth->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  pc_color->header.stamp    =  time_st.toNSec()/1e3;
  featurespcl_pub.publish(pc_color);

   
  //marker.action = visualization_msgs::Marker::DELETEALL; 


  sensor_msgs::ImagePtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered_image).toImageMsg();
  image_msg->header.stamp     = in_RGB->header.stamp;
 
  featuresRGB_pub.publish(image_msg);
  auto t2= Clock::now();
  std::cout<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()/1000000.0<<std::endl;
  /*for (int i =0;i<9;i++)
    std::cout<<Rdc(i)<<", "<<std::endl;*/


}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "pontCloudOntImage");
  ros::NodeHandle nh;  
  
  /// Load Parameters
  std::cout<<"Features LidarCamera Node initialized... :)";

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/angle_vertical_lines", angLin);
  nh.getParam("/imgTopic", imgTopic);
  nh.getParam("/depthTopic", depthTopic);
  nh.getParam("/odom_dron", odom_dron);
  nh.getParam("/real_flag", real_flag);
  nh.getParam("/tfPCL", tf_pcl);
  nh.getParam("/tf_word_or_sensor", tf_wOs);
  
    
  XmlRpc::XmlRpcValue param;

  nh.getParam("/lidarCamera_params/tdc", param);
  Tdc <<  (double)param[0]
         ,(double)param[1]
         ,(double)param[2];

  nh.getParam("/lidarCamera_params/rdc", param);

  Rdc <<  (double)param[0] ,(double)param[1] ,(double)param[2]
         ,(double)param[3] ,(double)param[4] ,(double)param[5]
         ,(double)param[6] ,(double)param[7] ,(double)param[8];

  nh.getParam("/lidarCamera_params/camera_matrix", param);

  Mc  <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3]
         ,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7]
         ,(double)param[8] ,(double)param[9] ,(double)param[10],(double)param[11];


  message_filters::Subscriber<Image>       rgbCam_sub (nh, imgTopic, 5);
  message_filters::Subscriber<PointCloud2> lidar_sub(nh, depthTopic, 5);
  message_filters::Subscriber<Odometry>    odom_robot(nh, odom_dron, 5);

  typedef sync_policies::ApproximateTime<Image, PointCloud2, Odometry> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgbCam_sub, lidar_sub, odom_robot);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  featuresRGB_pub = nh.advertise<sensor_msgs::Image>("/imgFeatures", 10);
  featurespcl_pub= nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/pcl_features", 10);

  ros::spin();
}