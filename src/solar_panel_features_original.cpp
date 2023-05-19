#include <ros/ros.h>
#include <limits>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
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

// #include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <chrono> 

// ransac
// #include <pcl/console/parse.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
// #include <pcl/sample_consensus/sac_model_sphere.h>

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

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

//Publisher
ros::Publisher featuresRGB_pub;   // features in RGB image
ros::Publisher featurespcl_pub;   // features in pcl
ros::Publisher path_pub;          // path coordinates 

// topics a suscribirse del nodo
std::string imgTopic   = "/camera/color/image_raw";
std::string depthTopic = "/camera/aligned_depth_to_color/image_raw";
std::string odom_dron  = "/dji_sdk/odometry";
std::string tf_pcl     = "camera_link";
bool tf_wOs = true;  // true mundo,  false sensor

float minlen = 0.5;
float maxlen = 10.0;
float angLin = 10.0;

int minCan = 80;
int maxCan = 100;

// matrices de calibracion entre la camara y el dron
Eigen::MatrixXf Tdc(3,1); // translation matrix dron-camera
Eigen::MatrixXf Rdc(3,3); // rotation matrix dron-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix


///////////////////////////////////////callback
void callback(const ImageConstPtr& in_RGB , const ImageConstPtr& in_depth, const OdometryConstPtr& odom_msg)
// void callback(const ImageConstPtr& in_RGB , const ImageConstPtr& in_depth)
{
      // Acceder a los datos del mensaje de Odometry
  // const double x_orient_odom = odom_msg->pose.pose.orientation.x;
  // const double y_orient_odom = odom_msg->pose.pose.orientation.y;
  // const double z_orient_odom = odom_msg->pose.pose.orientation.z;
  // const double w_orient_odom = odom_msg->pose.pose.orientation.w;

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

    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 4; j++) {            
    //        std::cout<<T_odom(i,j)<<",";
    //     }
    //     std::cout<<std::endl;
    // }
  


  auto t1 = Clock::now();

    cv_bridge::CvImagePtr cv_rgbCam , cv_depthCam;
        try
        {
          cv_rgbCam   = cv_bridge::toCvCopy(in_RGB, sensor_msgs::image_encodings::BGR8);         // imagen de color de la camara 
          cv_depthCam = cv_bridge::toCvCopy(in_depth, sensor_msgs::image_encodings::TYPE_16UC1);  // imagen de profundida de la camara 
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

  cv::Mat img_rgbCam    = cv_rgbCam->image;
  cv::Mat img_depthCam_IN  = cv_depthCam->image;
  cv::Mat depth_img  = cv_depthCam->image;


  // // correccion de distorcion
  // cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  // K.at<double>(0, 0) = (float)Mc(0,0);
  // K.at<double>(1, 1) = (float)Mc(1,1);
  // K.at<double>(0, 2) = (float)Mc(0,2);
  // K.at<double>(1, 2) = (float)Mc(1,2);
  // K.at<double>(2, 2) = 1.0;

  // cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
  // distortion_coefficients.at<double>(0, 0) =  0.090684;
  // distortion_coefficients.at<double>(0, 1) = -0.168557;
  // distortion_coefficients.at<double>(0, 2) =  0.000860;
  // distortion_coefficients.at<double>(0, 3) = -0.003330;
  // distortion_coefficients.at<double>(0, 4) =  0.000000;

  // cv::Mat corrected_image;
  // cv::undistort(depth_img, corrected_image, K, distortion_coefficients);

  // depth_img = corrected_image;

  // cv::undistort(img_rgbCam, corrected_image, K, distortion_coefficients);
  // img_rgbCam = corrected_image;




  int cols_img = in_depth->width;
  int rows_img = in_depth->height;

  cv::Mat imgGrayscale;        // grayscale of input image
  cv::Mat imgBlurred;            // intermediate blured image
  cv::Mat imgCanny;            // Canny edge image


  cv::cvtColor(img_rgbCam, imgGrayscale, CV_BGR2GRAY);        // convert to grayscale


   cv::Mat image_grayscale = depth_img.clone();
   image_grayscale.convertTo(image_grayscale, CV_8U, 1 / 256.0);
   image_grayscale = image_grayscale;
  //cv::cvtColor(depth_img, imgGrayscale2, CV_BGR2GRAY);        // convert to grayscale

  cv::GaussianBlur(image_grayscale,            // input image
      imgBlurred,                            // output image
      cv::Size(5, 5),                        // smoothing window width and height in pixels
      1.5);                                // sigma value, determines how much the image will be blurred

  cv::Canny(image_grayscale,            // input image
      imgCanny,                    // output image
      minCan,                        // low threshold
      maxCan);                        // high threshold


  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(imgCanny, lines, 1, CV_PI/180, 100, 0, 0);

  cv::Mat filtered_image = cv::Mat::zeros(rows_img, cols_img, CV_8UC1);

 
  for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        float angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180 / CV_PI;
        if (std::abs(angle) < 135 && std::abs(angle) > 45 ) {
            cv::line(filtered_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }

  }

  cv::Mat filtered_imageBN;
  cv::threshold(imgCanny, filtered_imageBN, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::normalize(filtered_imageBN, filtered_imageBN, 0, 1, cv::NORM_MINMAX);

  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "camera_link";   

  // sensor_msgs::Joy msg;
  // msg.header.stamp = ros::Time::now(); 
  // msg.header.frame_id = "camera_link";  

  Eigen::MatrixXf RTdc(4,4); // transformation matrix dron-camera
  RTdc<<   Rdc(0), Rdc(1) , Rdc(2) ,Tdc(0)
          ,Rdc(3), Rdc(4) , Rdc(5) ,Tdc(1)
          ,Rdc(6), Rdc(7) , Rdc(8) ,Tdc(2)
          ,0       , 0        , 0  , 1    ;


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_out (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_var (new pcl::PointCloud<pcl::PointXYZRGB>);
  point_var->width = 1;
  point_var->height = 1;
  point_var->is_dense = false;
  point_var->points.resize (point_var->width * point_var->height);

  // average camera depth
  float aver_depth = 0;
  float cont_aver = 0 ;

  for (int ic=0; ic< rows_img; ic++){
      for (int jc=0; jc<cols_img ; jc++){
        auto px = depth_img.at<ushort>(ic,jc);
        //auto px   =    depth_img.at<uchar>(ic,jc);

        auto cond = filtered_imageBN.at<uchar>(ic,jc);

        if ( (int)(cond) ==0 || (int)px ==0)
          continue;

          float z_feat = (int)px / 1000.0;
          //float z_feat = (int)px * (maxlen-minlen)/ 65536.0 + minlen;
       // else
         // z_feat = (int)px * (maxlen-minlen)/ 65536.0 + minlen;
        

        
        //if ((z_feat >maxlen*0.95  || z_feat < 2.5 )&& not(real_flag))
          //continue;

        // if (z_feat > 5.0)
         // z_feat = T_odom(2, 3);
         // continue;
      

        float u = (float)jc;
        float v = (float)ic;
        float fx = (float)Mc(0,0);
        float fy = (float)Mc(1,1);
        float cx = (float)Mc(0,2);
        float cy = (float)Mc(1,2);
        float x_feat = ((u-cx)) * z_feat /fx; 
        float y_feat = ((v-cy)) * z_feat /fy;

        Eigen::MatrixXf pc_matrix(4,1);
        Eigen::MatrixXf points_cam2dron(4,1);
        pc_matrix(0,0) = x_feat;
        pc_matrix(1,0) = y_feat;
        pc_matrix(2,0) = z_feat;
        pc_matrix(3,0) = 1.0;

        //T_odom(2, 3) = z_feat;

        points_cam2dron = pc_matrix;
       
        
        
        if(tf_wOs)
          points_cam2dron = T_odom*(RTdc*pc_matrix);
        //
        /*msg.axes.push_back(points_cam2dron(0,0));
        msg.axes.push_back(points_cam2dron(1,0));
        msg.axes.push_back(points_cam2dron(2,0));*/
      
        //joy_pub.publish(msg);

        cv::Vec3b color = img_rgbCam.at<cv::Vec3b>(ic,jc);

        point_var->points[0].x = points_cam2dron(0,0);
        point_var->points[0].y = points_cam2dron(1,0);
        point_var->points[0].z = points_cam2dron(2,0);
        point_var->points[0].r = (int)color[2];
        point_var->points[0].g = (int)color[1];
        point_var->points[0].b = (int)color[0];
        feature_out->push_back(point_var->points[0]);


       /* marker.header.stamp = ros::Time();
        marker.id = cont;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = points_cam2dron(0,0);
        marker.pose.position.y = points_cam2dron(1,0);
        marker.pose.position.z = points_cam2dron(2,0);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;*/
        //vis_pub.publish( marker );  

        aver_depth+=z_feat;
        cont_aver++;

      
      }
  }
  aver_depth = aver_depth/cont_aver;

  //std::cout<<"profundidad: "<<aver_depth<<std::endl;

  // features point cloud
  feature_out->is_dense = false;
  feature_out->width = feature_out->width;
  feature_out->height = feature_out->height;
  feature_out->points.resize (feature_out->width * feature_out->height);
  feature_out->header.frame_id = tf_pcl;
  ros::Time time_st = in_depth->header.stamp; // Para PCL se debe modificar el stamp y no se puede usar directamente el del topic de entrada
  feature_out->header.stamp    =  time_st.toNSec()/1e3;
  featurespcl_pub.publish(feature_out);
   
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

  ros::init(argc, argv, "panelFeatures");
  ros::NodeHandle nh;  
  
  /// Load Parameters
  std::cout<<"Features Node initialized... :)";

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/angle_vertical_lines", angLin);
  nh.getParam("/imgTopic", imgTopic);
  nh.getParam("/depthTopic", depthTopic);
  nh.getParam("/odom_dron", odom_dron);
  nh.getParam("/tfPCL", tf_pcl);
  nh.getParam("/tf_word_or_sensor", tf_wOs);

  nh.getParam("/minCan", minCan);
  nh.getParam("/maxCan", maxCan);
  
    
  XmlRpc::XmlRpcValue param;

  nh.getParam("/vision_params/tdc", param);
  Tdc <<  (double)param[0]
         ,(double)param[1]
         ,(double)param[2];

  nh.getParam("/vision_params/rdc", param);


  Rdc <<  (double)param[0] ,(double)param[1] ,(double)param[2]
         ,(double)param[3] ,(double)param[4] ,(double)param[5]
         ,(double)param[6] ,(double)param[7] ,(double)param[8];

  nh.getParam("/vision_params/camera_matrix", param);

  Mc  <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3]
         ,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7]
         ,(double)param[8] ,(double)param[9] ,(double)param[10],(double)param[11];


  message_filters::Subscriber<Image>  rgbCam_sub (nh, imgTopic, 5);
  message_filters::Subscriber<Image>  depthCam_sub(nh, depthTopic, 5);
  message_filters::Subscriber<Odometry>     odom_robot(nh, odom_dron, 5);

  typedef sync_policies::ApproximateTime<Image, Image, Odometry> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgbCam_sub, depthCam_sub, odom_robot);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  // typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgbCam_sub, depthCam_sub);
  // sync.registerCallback(boost::bind(&callback, _1, _2));

  featuresRGB_pub = nh.advertise<sensor_msgs::Image>("/imgFeatures", 10);
  featurespcl_pub= nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/pcl_features", 10);
 
  ros::spin();
  //return 0;
}