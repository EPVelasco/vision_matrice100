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
ros::Publisher panelFeatures_pub;   // features mask image
ros::Publisher panel_w_LinesFeatures_pub;   // features in RGB image
ros::Publisher panel_hsvfilter;   // features in RGB image

// topics a suscribirse del nodo
std::string imgTopic   = "/camera/aligned_depth_to_color/image_raw";

float rho = 1;
float theta = 90;
float threshold = 50;
float minLineLen = 50;
float maxLineGap = 10;
float minCan = 20;
float maxCan = 50;
float ang_t = 0;
float area_filter = 800.0; // 800 para real, 40 para simulado (tambien se modifica en el launch)
bool real_sim = true;  // (real == True) (sim == False)

void callback(const ImageConstPtr& in_depth)
{
  auto t1 = Clock::now();
  ros::Time start_time = ros::Time::now();

  cv_bridge::CvImagePtr cv_dCam;
      try
      {
        cv_dCam = cv_bridge::toCvCopy(in_depth, sensor_msgs::image_encodings::TYPE_16UC1);  // imagen de profundida de la camara 
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

  cv::Mat depth_img  = cv_dCam->image;
  cv::Mat image_grayscale = depth_img.clone();
  image_grayscale.convertTo(image_grayscale, CV_8U, 1/256.0);

  
  // Obtener las dimensiones de la imagen
  int imageHeight = in_depth->height; // rows
  int imageWidth = in_depth->width; // cols
 
  // Definir el rango de grises
  int umbralMin;
  int umbralMax;


  if (real_sim) // bandera para escoger entre la simulacion y el real (real == True) (sim == False)
  {
    umbralMin = 20;
    umbralMax = 255;
  }
  else
  {
    umbralMin = 0;
    umbralMax = 200;

  }

  // Binarizar la imagen utilizando el rango predefinido
  cv::Mat binaryImage;
  cv::threshold(image_grayscale, binaryImage, umbralMin, umbralMax, cv::THRESH_BINARY);



  // Buscar los contornos en la imagen binaria
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
 
  double areaThreshold = area_filter; // Ajusta el umbral de área según tus necesidades
  // Crear una imagen de salida para mostrar los contornos filtrados
  cv::Mat outputImage = cv::Mat::zeros(binaryImage.size(), CV_8UC3);

  // Filtrar los contornos más pequeños
  std::vector<cv::Vec4f> lines_input;
  for (size_t i = 0; i < contours.size(); i++)
  {
      double area = cv::contourArea(contours[i]);
      if (area > areaThreshold)
      {
          // Dibujar los contornos filtrados en la imagen de salida
          cv::Scalar color(255, 255, 255); // Color verde
          cv::drawContours(outputImage, contours, static_cast<int>(i), color, 2) ;
      }
  }

  cv::Mat gray_image_filter;
  cv::cvtColor(outputImage, gray_image_filter, CV_BGR2GRAY);  

  // dilatacion vertical 
  cv::Mat kernel_dil = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 50)); // Kernel de 1x21 para la dilatación vertical
  cv::dilate(gray_image_filter, gray_image_filter, kernel_dil);
  // variables para detectar lineas con otro algoritmo

  //cv::cvtColor(gray_image_filter,outputImage, CV_GRAY2BGR);  

  std::vector<cv::Point>  contour_left;
  std::vector<cv::Point> contour_rigth;

  // Recorrer la imagen por filas y columnas
  for (int i = 100; i < imageHeight-100; i++) { // filas
      bool flag_left = false;
      bool flag_rigth = false;
      for (int j = 1; j < imageWidth/2; j++) { // columnas
          // Obtener el valor del píxel en (i, j)

          int pixel_left  = static_cast<int>(gray_image_filter.at<uchar>(i, j));
          int pixel_right = static_cast<int>(gray_image_filter.at<uchar>(i, imageWidth-j));

          if (pixel_left==255 && flag_left == false){
            contour_left.push_back(cv::Point(j, i));
            flag_left = true;
          }
          if (pixel_right==255 && flag_rigth == false){
            contour_rigth.push_back(cv::Point(imageWidth-j, i));
            flag_rigth = true;
          }
          if (flag_rigth && flag_left)
            break;
          
      }
  }

  cv::Mat mono_resultImage = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);

  std::vector<std::vector<cv::Point>> contours_2;
  contours_2.push_back(contour_left);

  // cv::drawContours(outputImage, std::vector<std::vector<cv::Point>>{contour_left}, 0, cv::Scalar(0, 0, 255), 2);
  
  contours_2.push_back(contour_rigth);

//   cv::drawContours(outputImage, std::vector<std::vector<cv::Point>>{contour_rigth}, 0, cv::Scalar(255, 0, 0), 2);

//  cv::line(outputImage,cv::Point(imageWidth/2,0 ), cv::Point(imageWidth/2,imageHeight), cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
//  cv::line(outputImage,cv::Point(0,100), cv::Point(imageWidth,100), cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
//  cv::line(outputImage,cv::Point(0,imageHeight-100), cv::Point(imageWidth,imageHeight-100), cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
   
  std::vector<cv::Vec4f> linesap;
  for (const auto& contour : contours_2) {
      // Ajustar una línea a los contornos
      if (contour.size()==0)
        break;
      cv::Vec4f line;
      cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);
      linesap.push_back(line); 
  }

  for (const auto& line : linesap) {
      float vx = line[0];
      float vy = line[1];
      float x = line[2];
      float y = line[3];
      
      //Puntos de la linea
      cv::Point pt1_out_lin(x - 1000 * vx, y - 1000 * vy);
      cv::Point pt2_out_lin(x + 1000 * vx, y + 1000 * vy); 
      float angle = std::atan2(pt1_out_lin.y - pt2_out_lin.y, pt1_out_lin.x - pt2_out_lin.x) * 180 / CV_PI;

      if (std::abs(angle) < 135 && std::abs(angle) > 45 ) {
        cv::line(mono_resultImage, pt1_out_lin, pt2_out_lin, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        cv::line(depth_img, pt1_out_lin, pt2_out_lin, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
      }
   }
  
  ros::Time end_time = ros::Time::now();  
  // Calcular la diferencia de tiempo
  ros::Duration delay_ros = end_time - start_time;
  
  sensor_msgs::ImagePtr image_msg_mask;
  image_msg_mask = cv_bridge::CvImage(std_msgs::Header(), "mono8", mono_resultImage).toImageMsg();
  image_msg_mask->header = in_depth->header;
  image_msg_mask->header.stamp = in_depth->header.stamp + delay_ros;
  panelFeatures_pub.publish(image_msg_mask);  

  sensor_msgs::ImagePtr image_msg_rgb;
  image_msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_img).toImageMsg();
  image_msg_rgb->header = in_depth->header;
  image_msg_rgb->header.stamp = in_depth->header.stamp + delay_ros;
  panel_w_LinesFeatures_pub.publish(image_msg_rgb);  

  sensor_msgs::ImagePtr image_msg_bn;
  image_msg_bn = cv_bridge::CvImage(std_msgs::Header(), "mono8", binaryImage).toImageMsg();
  image_msg_bn->header = in_depth->header;
  image_msg_bn->header.stamp = in_depth->header.stamp + delay_ros;
  panel_hsvfilter.publish(image_msg_bn);  

  auto t10= Clock::now();
  std::cout<<"time total (ms): "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t10-t1).count()/1000000.0<<std::endl;
  
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "panelFeatures_real");
  ros::NodeHandle nh;  

  nh.getParam("/rho", rho);
  nh.getParam("/theta", theta);
  nh.getParam("/threshold", threshold);
  nh.getParam("/minLineLen", minLineLen);
  nh.getParam("/maxLineGap", maxLineGap);
  nh.getParam("/minCan", minCan);
  nh.getParam("/maxCan", maxCan);
  nh.getParam("/imgTopic", imgTopic);
  nh.getParam("/area_filter", area_filter); 
  nh.getParam("/real_sim",real_sim);
  
  /// Load Parameters
  std::cout<<"Panel Image mask Real initialized... :)";
  if (real_sim)
    std::cout<<"Modo Real";
  else
    std::cout<<"Modo Simulacion";

  ros::Subscriber sub = nh.subscribe<Image>(imgTopic, 1, callback);
  panelFeatures_pub         = nh.advertise<sensor_msgs::Image>("/panel/image/mask", 1);  
  panel_w_LinesFeatures_pub = nh.advertise<sensor_msgs::Image>("/panel/image/rgb_mask", 1);  
  panel_hsvfilter = nh.advertise<sensor_msgs::Image>("/panel/image/hsv_mask", 1);  
  ros::spin();
}