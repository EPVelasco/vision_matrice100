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


// images publish
cv::Mat mono_resultImage = cv::Mat::zeros(480, 640, CV_8UC1);
cv::Mat gray_image_filter;
cv::Mat rgb_image;



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
float area_filter = 800.0; // 800 para real, 40 para simulado (tambien se modifica en el launch)
bool real_sim = true;  // (real == True) (sim == False)

ros::Time start_time ;


cv::Mat thinning(const cv::Mat& input)
{
    cv::Mat skel(input.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do {
        cv::erode(input, eroded, element);
        cv::dilate(eroded, temp, element);
        cv::subtract(input, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(input);

        done = (cv::countNonZero(input) == 0);
    } while (!done);

    return skel;
}


void callback(const ImageConstPtr& in_image)
{
  auto t1 = Clock::now();
  start_time = ros::Time::now();

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

 rgb_image  = cv_rgbCam->image;

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
  cv::Scalar lowerWhite;
  cv::Scalar upperWhite;

  if (real_sim) // bandera para escoger entre la simulacion y el real (real == True) (sim == False)
  {
    lowerWhite = cv::Scalar(0, 0, 250);  // Umbral inferior para blanco
    upperWhite = cv::Scalar(180, 100, 255);  // Umbral superior para blanco
    std::cout<<"********Filtrado en REAL** ";
  }
  else
  {
    lowerWhite = cv::Scalar(0, 0, 200);  // Umbral inferior para blanco
    upperWhite = cv::Scalar(180, 30, 255);  // Umbral superior para blanco
    std::cout<<"********Filtrado en SIM** ";
  }



  // Definir el rango de colores blanco en HSV
  


  // Binarizar la imagen utilizando el rango de colores blanco
  cv::Mat binaryImage;
  cv::inRange(hsvImage, lowerWhite, upperWhite, binaryImage);

  // Buscar los contornos en la imagen binaria
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
  // Definir el umbral de área para filtrar los contornos
 // double areaThreshold = 800.0; // Ajusta el umbral de área según tus necesidades

  // auto t2= Clock::now();
  // std::cout<<"time find contotours: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()/1000000.0<<std::endl;

  double areaThreshold = area_filter; // Ajusta el umbral de área según tus necesidades
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
          cv::drawContours(outputImage, contours, static_cast<int>(i), color, 2) ;
      }
  }


  // auto t3= Clock::now();
  // std::cout<<"time filtrado areas: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count()/1000000.0<<std::endl;


 
  cv::cvtColor(outputImage, gray_image_filter, CV_BGR2GRAY);  


  //cv::Mat skel = thinning(gray_image_filter);

  // Definir el elemento estructurante para la operación de cierre
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); // Ajusta el tamaño del kernel según tus necesidades

  // Realizar la operación de cierre para completar el espacio entre líneas
  cv::Mat closedImage;
  cv::morphologyEx(gray_image_filter, closedImage, cv::MORPH_CLOSE, kernel);

  // // Definir la región de interés en la mitad inferior de la imagen
  // cv::Rect roi(0, 0, imageWidth, imageHeight / 2);
  // // Rellenar la región de interés con ceros
  // gray_image_filter(roi) = 0;

  // Redimensionar la imagen
  cv::Size nuevoTamano(imageWidth/8, imageHeight/8);
  cv::Mat imagenRedimensionada;
  cv::resize(gray_image_filter, imagenRedimensionada, nuevoTamano);

  // Binarizar la imagen
  cv::threshold(imagenRedimensionada, imagenRedimensionada, 0, 255, cv::THRESH_BINARY);



  cv::Mat filtered_image = cv::Mat::zeros(imageHeight/8, imageWidth/8, CV_8UC1);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(imagenRedimensionada, lines, rho,  theta, threshold, minLineLen, maxLineGap); //rho, theta, threshold, minLineLen, maxLineGap

  // float xc = 0 , yc = 0;
  // cv::Vec4i lin_group1(0, 0, 0, 0);
  // cv::Vec4i lin_group2(0, 0, 0, 0);
  // int contleft = 0;
  // int contrigth = 0;

  // auto t4= Clock::now();
  // std::cout<<"time hough lineas: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count()/1000000.0<<std::endl;

  float line1_len =0;
  float line2_len = 0;
  cv::Vec4i line1_max (0, 0, 0, 0);
  cv::Vec4i line2_max (0, 0, 0, 0);

  for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        float angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180 / CV_PI;
      if (std::abs(angle) < 120 && std::abs(angle) > 60 ) {
        //cv::line(filtered_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        if((line[0]<imagenRedimensionada.cols/2 and line[2]<imagenRedimensionada.cols/2)) {

          // lin_group1[0] = line[0] + lin_group1[0];
          // lin_group1[1] = line[1] + lin_group1[1];
          // lin_group1[2] = line[2] + lin_group1[2];
          // lin_group1[3] = line[3] + lin_group1[3];
          // contleft++;


          // linea mas larga izquierda
          float len_1 = sqrt(pow(line[0]-line[2],2)+pow(line[1]-line[3],2));
          if (len_1>line1_len){
            line1_len = len_1;
            line1_max = line;
          }


        }
        else{
          // lin_group2[0] = line[0] + lin_group2[0];
          // lin_group2[1] = line[1] + lin_group2[1];
          // lin_group2[2] = line[2] + lin_group2[2];
          // lin_group2[3] = line[3] + lin_group2[3];
          // contrigth++;

          // linea mas larga derecha
          float len_2 = sqrt(pow(line[0]-line[2],2)+pow(line[1]-line[3],2));
          if (len_2>line2_len){
            line2_len = len_2;
            line2_max = line;
          }
        }
    }
  }

  // auto t5= Clock::now();
  // std::cout<<"time bests lineas: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t5-t4).count()/1000000.0<<std::endl;


  // promedio de las lineas

  // std::cout<<"contleft: "<<contleft<<std::endl;
  // std::cout<<"contrigth: "<<contrigth<<std::endl;
  // std::cout<<"Line1 :"<< lin_group1<<std::endl;
  // std::cout<<"Line2 :"<< lin_group2<<std::endl;


  // if (contleft >0 and contrigth >0 ){

  //   lin_group1[0] = lin_group1[0]/contleft;
  //   lin_group1[1] = lin_group1[1]/contleft;
  //   lin_group1[2] = lin_group1[2]/contleft;
  //   lin_group1[3] = lin_group1[3]/contleft;

  //   lin_group2[0] = lin_group2[0]/contrigth;
  //   lin_group2[1] = lin_group2[1]/contrigth;
  //   lin_group2[2] = lin_group2[2]/contrigth;
  //   lin_group2[3] = lin_group2[3]/contrigth;



  //   // float angle_lin1 =  std::atan2(lin_group1[3] - lin_group1[1], lin_group1[2] - lin_group1[0])*180/CV_PI;
  //   // float angle_lin2 =  std::atan2(lin_group2[3] - lin_group2[1], lin_group2[2] - lin_group2[0])*180/CV_PI;
  //   // angle_lin2 = angle_lin2 -180;

  //   // xc = (lin_group1[0] + lin_group1[2]+lin_group2[0] + lin_group2[2])/4;
  //   // yc = imageHeight/2;

  //  // xc = (max_x+min_x)/2;
  //   // ang_t = (angle_lin1+angle_lin2)/2;

  //   // std::cout<<"Angle degree: " << ang_t<<std::endl;
  //   // ang_t = ang_t*CV_PI/180;
  //   // std::cout<<"Angle rad: " << ang_t<<std::endl;
  //   // std::cout<<"xc : " << xc<<std::endl;
  // }
  // else{
  //   // xc = 0;
  //   // yc = 0;
  //   std::cout<<"[ERROR en deteccion de panel]: no hay suficientes lineas para deteccion"<<std::endl;
  // }

  // auto t6= Clock::now();
  // std::cout<<"not plot lineas: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t6-t5).count()/1000000.0<<std::endl;


  cv::Mat resultImage;
  cv::cvtColor(filtered_image, resultImage, cv::COLOR_GRAY2BGR);

  // cv::line(resultImage, cv::Point(lin_group1[0], lin_group1[1]), cv::Point(lin_group1[2], lin_group1[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  // cv::line(resultImage, cv::Point(lin_group2[0], lin_group2[1]), cv::Point(lin_group2[2], lin_group2[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  cv::line(resultImage, cv::Point(line1_max[0], line1_max[1]), cv::Point(line1_max[2], line1_max[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::line(resultImage, cv::Point(line2_max[0], line2_max[1]), cv::Point(line2_max[2], line2_max[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  // Redimensionar la imagen
  cv::Size resIm(imageWidth, imageHeight);
  cv::Mat imagen_reestablecida;
  cv::resize(resultImage, imagen_reestablecida, resIm);

    // Binarizar la imagen
  cv::threshold(imagen_reestablecida, imagen_reestablecida, 0, 255, cv::THRESH_BINARY);

  cv::Mat mono_img_result;
  cv::cvtColor(imagen_reestablecida, mono_img_result, cv::COLOR_BGR2GRAY);

  
  std::vector<std::vector<cv::Point>> contours_2;
  cv::findContours(mono_img_result, contours_2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // auto t7= Clock::now();
  // std::cout<<"plot lineas: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t7-t6).count()/1000000.0<<std::endl;


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
      cv::line(rgb_image, pt1_out_lin, pt2_out_lin, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
   }
 
  // auto t8= Clock::now();
  // std::cout<<"lasts plot lineas: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t8-t7).count()/1000000.0<<std::endl;
  

  // auto t9= Clock::now();
  // std::cout<<"time publish: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t9-t8).count()/1000000.0<<std::endl;

  auto t10= Clock::now();
  std::cout<<"time total: "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t10-t1).count()/1000000.0<<std::endl;

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

  ros::Subscriber sub = nh.subscribe<Image>(imgTopic, 1, callback);
  panelFeatures_pub         = nh.advertise<sensor_msgs::Image>("/panel/image/mask", 1);  
  panel_w_LinesFeatures_pub = nh.advertise<sensor_msgs::Image>("/panel/image/rgb_mask", 1);  
  panel_hsvfilter = nh.advertise<sensor_msgs::Image>("/panel/image/hsv_mask", 1);  

  ros::Rate loopRate(15);
  while(ros::ok()){
    ros::spinOnce();
    ros::Time end_time = ros::Time::now();  
    // Calcular la diferencia de tiempo
    ros::Duration delay_ros = end_time - start_time;
    
    sensor_msgs::ImagePtr image_msg_mask;
    image_msg_mask = cv_bridge::CvImage(std_msgs::Header(), "mono8", mono_resultImage).toImageMsg();
  // image_msg_mask->header = in_image->header;
    image_msg_mask->header.stamp = ros::Time::now() + delay_ros;
    panelFeatures_pub.publish(image_msg_mask);  


    sensor_msgs::ImagePtr image_msg_rgb;
    image_msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
    //image_msg_rgb->header = in_image->header;
    image_msg_rgb->header.stamp = ros::Time::now() + delay_ros;
    panel_w_LinesFeatures_pub.publish(image_msg_rgb);  

    sensor_msgs::ImagePtr image_msg_hsv;
    image_msg_hsv = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_image_filter).toImageMsg();
  // image_msg_hsv->header = in_image->header;
    image_msg_hsv->header.stamp = ros::Time::now() + delay_ros;
    panel_hsvfilter.publish(image_msg_hsv);  

  loopRate.sleep();
    
  } 

  return 0;
}