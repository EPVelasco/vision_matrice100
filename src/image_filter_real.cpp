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
std::string imgTopic   = "/camera/color/image_raw";

float rho = 1;
float theta = 90;
float threshold = 50;
float minLineLen = 50;
float maxLineGap = 10;
float minCan = 20;
float maxCan = 50;
float ang_t = 0;

void callback(const ImageConstPtr& in_image)
{

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
  int imageHeight = in_image->height;
  int imageWidth = in_image->width;

  // cv::Mat imgCanny;            // Canny edge image 

  // cv::Mat image_grayscale = rgb_image.clone();
  // cv::cvtColor(rgb_image, image_grayscale, CV_BGR2GRAY);        // convert to grayscale
  // cv::GaussianBlur(image_grayscale, image_grayscale, cv::Size(5, 5), 0);

  // cv::Canny(image_grayscale,        // input image
  //     imgCanny,                    // output image
  //     minCan,                      // low threshold
  //     maxCan);                     // high threshold

  // // Definir el color de los bordes a resaltar (claro)
  // cv::Scalar edgeColor(255, 255, 255);  // Blanco

  // // Crear una imagen en blanco del mismo tamaño que la original
  // cv::Mat filteredImage(rgb_image.size(), CV_8UC3, cv::Scalar(0, 0, 0));

  // // Copiar los píxeles del panel solar original a la imagen filtrada, resaltando los bordes
  // for (int y = 0; y < imageHeight; y++)
  // {
  //     for (int x = 0; x < imageWidth; x++)
  //     {
  //         if (imgCanny.at<uchar>(y, x) > 0)
  //         {
  //             filteredImage.at<cv::Vec3b>(y, x) = cv::Vec3b(edgeColor[0], edgeColor[1], edgeColor[2]);
  //         }
  //     }
  // }

  cv::Mat hsvImage;
  cv::cvtColor(rgb_image, hsvImage, cv::COLOR_BGR2HSV);

  // Definir el rango de colores blanco en HSV
  cv::Scalar lowerWhite(0, 0, 200);  // Umbral inferior para blanco
  cv::Scalar upperWhite(180, 30, 255);  // Umbral superior para blanco

  // Binarizar la imagen utilizando el rango de colores blanco
  cv::Mat binaryImage;
  cv::inRange(hsvImage, lowerWhite, upperWhite, binaryImage);

  // Buscar los contornos en la imagen binaria
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  // Definir el umbral de área para filtrar los contornos
  double areaThreshold = 800.0; // Ajusta el umbral de área según tus necesidades

  // Crear una imagen de salida para mostrar los contornos filtrados
  cv::Mat outputImage = cv::Mat::zeros(binaryImage.size(), CV_8UC3);

  // Filtrar los contornos más pequeños
  for (size_t i = 0; i < contours.size(); i++)
  {
      double area = cv::contourArea(contours[i]);
      if (area > areaThreshold)
      {
          // Dibujar los contornos filtrados en la imagen de salida
          cv::Scalar color(255, 255, 255); // Color verde
          cv::drawContours(outputImage, contours, static_cast<int>(i), color, 2);
      }
  }

  cv::Mat gray_image_filter;
  cv::cvtColor(outputImage, gray_image_filter, CV_BGR2GRAY);  

  // Definir el elemento estructurante para la operación de cierre
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); // Ajusta el tamaño del kernel según tus necesidades

  // Realizar la operación de cierre para completar el espacio entre líneas
  cv::Mat closedImage;
  cv::morphologyEx(gray_image_filter, closedImage, cv::MORPH_CLOSE, kernel);

  cv::Mat filtered_image = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(closedImage, lines, rho,  theta, threshold, minLineLen, maxLineGap); //rho, theta, threshold, minLineLen, maxLineGap

  // float xc = 0 , yc = 0;
  cv::Vec4i lin_group1(0, 0, 0, 0);
  cv::Vec4i lin_group2(0, 0, 0, 0);
  int contleft = 0;
  int contrigth = 0;

  for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        float angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180 / CV_PI;
      if (std::abs(angle) < 120 && std::abs(angle) > 60 ) {
        //cv::line(filtered_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        if((line[0]<imageWidth/2 and line[2]<imageWidth/2)) {

          lin_group1[0] = line[0] + lin_group1[0];
          lin_group1[1] = line[1] + lin_group1[1];
          lin_group1[2] = line[2] + lin_group1[2];
          lin_group1[3] = line[3] + lin_group1[3];
          contleft++;

        }
        else{
          lin_group2[0] = line[0] + lin_group2[0];
          lin_group2[1] = line[1] + lin_group2[1];
          lin_group2[2] = line[2] + lin_group2[2];
          lin_group2[3] = line[3] + lin_group2[3];
          contrigth++;
        }
    }
  }

  // promedio de las lineas

  // std::cout<<"contleft: "<<contleft<<std::endl;
  // std::cout<<"contrigth: "<<contrigth<<std::endl;
  // std::cout<<"Line1 :"<< lin_group1<<std::endl;
  // std::cout<<"Line2 :"<< lin_group2<<std::endl;


  if (contleft >0 and contrigth >0 ){

    lin_group1[0] = lin_group1[0]/contleft;
    lin_group1[1] = lin_group1[1]/contleft;
    lin_group1[2] = lin_group1[2]/contleft;
    lin_group1[3] = lin_group1[3]/contleft;

    lin_group2[0] = lin_group2[0]/contrigth;
    lin_group2[1] = lin_group2[1]/contrigth;
    lin_group2[2] = lin_group2[2]/contrigth;
    lin_group2[3] = lin_group2[3]/contrigth;

    // float angle_lin1 =  std::atan2(lin_group1[3] - lin_group1[1], lin_group1[2] - lin_group1[0])*180/CV_PI;
    // float angle_lin2 =  std::atan2(lin_group2[3] - lin_group2[1], lin_group2[2] - lin_group2[0])*180/CV_PI;
    // angle_lin2 = angle_lin2 -180;

    // xc = (lin_group1[0] + lin_group1[2]+lin_group2[0] + lin_group2[2])/4;
    // yc = imageHeight/2;

   // xc = (max_x+min_x)/2;
    // ang_t = (angle_lin1+angle_lin2)/2;

    // std::cout<<"Angle degree: " << ang_t<<std::endl;
    // ang_t = ang_t*CV_PI/180;
    // std::cout<<"Angle rad: " << ang_t<<std::endl;
    // std::cout<<"xc : " << xc<<std::endl;
  }
  else{
    // xc = 0;
    // yc = 0;
    std::cout<<"[ERROR en deteccion de panel]: no hay suficientes lineas"<<std::endl;
  }

  cv::Mat resultImage;
  cv::cvtColor(filtered_image, resultImage, cv::COLOR_GRAY2BGR);

  cv::line(resultImage, cv::Point(lin_group1[0], lin_group1[1]), cv::Point(lin_group1[2], lin_group1[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::line(resultImage, cv::Point(lin_group2[0], lin_group2[1]), cv::Point(lin_group2[2], lin_group2[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  cv::Mat mono_resultImage;
  cv::cvtColor(resultImage, mono_resultImage, cv::COLOR_BGR2GRAY);

  std::vector<std::vector<cv::Point>> contours_2;
  cv::findContours(mono_resultImage, contours_2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Vec4f> linesap;
  for (const auto& contour : contours_2) {
      // Ajustar una línea a los contornos
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
      cv::line(mono_resultImage, pt1_out_lin, pt2_out_lin, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
   }
  
  sensor_msgs::ImagePtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mono_resultImage).toImageMsg();
  image_msg->header = in_image->header;
  panelFeatures_pub.publish(image_msg);  
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
  
  /// Load Parameters
  std::cout<<"Panel Image mask Real initialized... :)";

  nh.getParam("/imgTopic", imgTopic);
  ros::Subscriber sub = nh.subscribe<Image>(imgTopic, 10, callback);
  panelFeatures_pub = nh.advertise<sensor_msgs::Image>("/panel/image/mask", 10);  
  ros::spin();
}