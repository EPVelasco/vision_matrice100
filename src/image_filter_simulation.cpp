#include <ros/ros.h>
#include <limits>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <chrono> 

typedef std::chrono::high_resolution_clock Clock;

using namespace Eigen;
using namespace sensor_msgs;
using namespace std;

//Publisher
ros::Publisher panelFeatures_pub;   // features in RGB image

// topics a suscribirse del nodo
std::string imgTopic   = "/camera/aligned_depth_to_color/image_raw";

int minCan = 20;
int maxCan = 50;

float minlenli = 1;
float maxlenli = 10;
float angle_desired = 0;
 
void callback(const ImageConstPtr& in_depth)
{
  auto t1 = Clock::now();

  cv_bridge::CvImagePtr cv_rgbCam , cv_depthCam;
      try
      {
        cv_depthCam = cv_bridge::toCvCopy(in_depth, sensor_msgs::image_encodings::TYPE_16UC1);  // imagen de profundida de la camara 
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

  cv::Mat depth_img  = cv_depthCam->image;

  // Obtener las dimensiones de la imagen
  int imageHeight = in_depth->height;
  int imageWidth = in_depth->width;

  cv::Mat imgCanny;            // Canny edge image 

  cv::Mat image_grayscale = depth_img.clone();
  image_grayscale.convertTo(image_grayscale, CV_8U, 1 / 256.0);

  cv::Canny(image_grayscale,        // input image
      imgCanny,                    // output image
      minCan,                      // low threshold
      maxCan);                     // high threshold


  cv::Mat filtered_image = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
  // Definir el kernel para la operación de dilatación
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

  // Aplicar la operación de dilatación a la imagen binarizada
  cv::dilate(imgCanny, imgCanny, kernel);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(imgCanny, lines, 1.0,  CV_PI/180, 100, 100.0, 0.0);
 
  for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        float angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180 / CV_PI;
        if (std::abs(angle) < 120 && std::abs(angle) > 60 ) {
            cv::line(filtered_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
        }
  }

  // Definir la región de interés en la mitad inferior de la imagen
  cv::Rect roi(0, 0, imageWidth, imageHeight / 2);
  // Rellenar la región de interés con ceros
  filtered_image(roi) = 0;

  cv::Mat kernel_dil = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 11));

  // Aplicar la operación de dilatación a la imagen binarizada
  cv::dilate(filtered_image, filtered_image, kernel_dil);

  sensor_msgs::ImagePtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered_image).toImageMsg();
  panelFeatures_pub.publish(image_msg);

  
  auto t2= Clock::now();
  std::cout<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()/1000000.0<<std::endl;

}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "panelFeatures");
  ros::NodeHandle nh;  
  
  /// Load Parameters
  std::cout<<"Panel Image mask Simulation initialized... :)";

  nh.getParam("/imgTopic", imgTopic);
  ros::Subscriber sub = nh.subscribe<Image>(imgTopic, 10, callback);
  panelFeatures_pub = nh.advertise<sensor_msgs::Image>("/panel/image/mask", 10);
  
 
  ros::spin();



}