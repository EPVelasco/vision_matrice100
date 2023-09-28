/*
Programa que extrae las caractersiticas del panel en el canal RGB lo que hace es filtrar medainte color los colores
blancos, estos valores son ajustables desde el launch. Despues de esto estos puntos los aproxima a lineas rectas
*/

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
#include "vision_matrice100/DurationStamped.h"
#include <std_msgs/Float32MultiArray.h>


typedef std::chrono::high_resolution_clock Clock;

using namespace Eigen;
using namespace sensor_msgs;
using namespace std;

//Publisher
ros::Publisher panelFeatures_pub;   // features mask image
ros::Publisher panel_w_LinesFeatures_pub;   // features in RGB image
ros::Publisher panel_hsvfilter;   // features in RGB image
ros::Publisher time_pub;          // tiempo de ejecucion

// topics a suscribirse del nodo
std::string imgTopic   = "/camera/color/image_raw";

float threshold = 50;
float ang_t = 0;
float area_filter = 800.0; // 800 para real, 40 para simulado (tambien se modifica en el launch)
bool real_sim = true;  // (real == True) (sim == False)

float h_low = 0;
float s_low = 0;
float v_low = 200;

float h_high = 179;
float s_high = 100;
float v_high = 255;

void callback_img(const std_msgs::Float32MultiArray &img_params){

  h_low = img_params.data[0];
  s_low = img_params.data[1];
  v_low = img_params.data[2];

  h_high = img_params.data[3];
  s_high = img_params.data[4];
  v_high = img_params.data[5];


  std::cout<<"***Parametros de filtro modificados***" <<std::endl;
  std::cout<<"h_low: " <<h_low <<std::endl;
  std::cout<<"s_low: " <<s_low <<std::endl;
  std::cout<<"v_low: " <<v_low <<std::endl;
  std::cout<<"h_high: " <<h_high <<std::endl;
  std::cout<<"s_high: " <<s_high <<std::endl;
  std::cout<<"v_high: " <<v_high <<std::endl;
}


void callback(const ImageConstPtr& in_image)
{
  auto t1 = Clock::now();
  ros::Time start_time = ros::Time::now();

  cv_bridge::CvImagePtr cv_rgbCam;
      try
      {
        cv_rgbCam = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::BGR8);  // imagen de profundida de la camara 
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

  cv::Mat rgb_image  = cv_rgbCam->image;
  
  // Obtener las dimensiones de la imagen
  int imageHeight = in_image->height; // rows
  int imageWidth = in_image->width; // cols
 
  // Definir el rango de colores blanco en HSV
  cv::Scalar lowerWhite;
  cv::Scalar upperWhite;

  if (real_sim) // bandera para escoger entre la simulacion y el real (real == True) (sim == False)
  {
    lowerWhite = cv::Scalar(h_low, s_low, v_low);  // Umbral inferior para blanco
    upperWhite = cv::Scalar(h_high, s_high, v_high);  // Umbral superior para blanco
  }
  else
  {
    lowerWhite = cv::Scalar(0, 0, 200);  // Umbral inferior para blanco
    upperWhite = cv::Scalar(180, 30, 255);  // Umbral superior para blanco
  }

  cv::Mat hsvImage;
  cv::cvtColor(rgb_image, hsvImage, cv::COLOR_BGR2HSV);

  // Define el rango superior para el color rojo en el espacio de color HSV (solo para preubas en escenario de pruebas)
  cv::Scalar lower2(160, s_low, v_low);
  cv::Scalar upper2(179, s_high, v_high);

  // Crea una máscara para los valores de color dentro del rango inferior
  cv::Mat lower_mask, upper_mask, full_mask;
  cv::inRange(hsvImage, lowerWhite, upperWhite, lower_mask);
  cv::inRange(hsvImage, lower2, upper2, upper_mask);
  full_mask = lower_mask + upper_mask;

  //full_mask = lower_mask;

  // Buscar los contornos en la imagen binaria
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(full_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  

  // Crear una imagen de salida para mostrar los contornos filtrados
  cv::Mat outputImage = cv::Mat::zeros(full_mask.size(), CV_8UC3);

  // Filtrar los contornos más pequeños
  std::vector<cv::Vec4f> lines_input;
  for (size_t i = 0; i < contours.size(); i++)
  {
      double area = cv::contourArea(contours[i]);
      if (area > area_filter)
      {
          // Dibujar los contornos filtrados en la imagen de salida
          cv::Scalar color(255, 255, 255); 
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
      cv::fitLine(contour, line, cv::DIST_L1, 0, 0.01, 0.01);
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
        cv::line(rgb_image, pt1_out_lin, pt2_out_lin, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
      }
   }
  
  ros::Time end_time = ros::Time::now();  
  // Calcular la diferencia de tiempo
  ros::Duration delay_ros = end_time - start_time;
  
  sensor_msgs::ImagePtr image_msg_mask;
  image_msg_mask = cv_bridge::CvImage(std_msgs::Header(), "mono8", mono_resultImage).toImageMsg();
  image_msg_mask->header = in_image->header;
  image_msg_mask->header.stamp = in_image->header.stamp + delay_ros;
  panelFeatures_pub.publish(image_msg_mask);  

  sensor_msgs::ImagePtr image_msg_rgb;
  image_msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
  image_msg_rgb->header = in_image->header;
  image_msg_rgb->header.stamp = in_image->header.stamp + delay_ros;
  panel_w_LinesFeatures_pub.publish(image_msg_rgb);  

  sensor_msgs::ImagePtr image_msg_hsv;
  image_msg_hsv = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_image_filter).toImageMsg();
  image_msg_hsv->header = in_image->header;
  image_msg_hsv->header.stamp = in_image->header.stamp + delay_ros;
  panel_hsvfilter.publish(image_msg_hsv);  

  auto t10= Clock::now();
  std::cout<<"time total (ms): "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t10-t1).count()/1000000.0<<std::endl;
    // mensaje de tiempo de ejecucion
  vision_matrice100::DurationStamped time_msg;  // Crear una instancia del mensaje
  time_msg.header.stamp = in_image->header.stamp + delay_ros;  // Asignar la marca de tiempo actual al encabezado
  time_msg.data = delay_ros;  // Asignar el valor a publicar al campo 'data' del mensaje

  time_pub.publish(time_msg);  // Publicar el mensaje
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "panelFeatures_real");
  ros::NodeHandle nh;  

  nh.getParam("/threshold", threshold);
  nh.getParam("/imgTopic", imgTopic);
  nh.getParam("/area_filter", area_filter); 
  nh.getParam("/real_sim",real_sim);
  nh.getParam("/h_low", h_low); 
  nh.getParam("/s_low",s_low);
  nh.getParam("/v_low", v_low); 
  nh.getParam("/h_high",h_high);
  nh.getParam("/s_high", s_high); 
  nh.getParam("/v_high",v_high);
 
  /// Load Parameters
  std::cout<<"Panel Image mask Real initialized... :)";
  if (real_sim)
    std::cout<<"Modo Real";
  else
    std::cout<<"Modo Simulacion";

  ros::Subscriber sub = nh.subscribe<Image>(imgTopic, 10, callback);
  ros::Subscriber sub_2 = nh.subscribe("/img_params", 10, callback_img); 

  panelFeatures_pub         = nh.advertise<sensor_msgs::Image>("/panel/image/mask", 10);  
  panel_w_LinesFeatures_pub = nh.advertise<sensor_msgs::Image>("/panel/image/rgb_mask", 10);  
  panel_hsvfilter = nh.advertise<sensor_msgs::Image>("/panel/image/hsv_mask", 10);  
  time_pub = nh.advertise<vision_matrice100::DurationStamped>("/panel/image/runtime", 10);  
  ros::spin();
}