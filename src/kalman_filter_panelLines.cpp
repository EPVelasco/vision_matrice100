#include <ros/ros.h>
#include <limits>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "vision_matrice100/DurationStamped.h"
#include "../include/utils_lib.h"


typedef std::chrono::high_resolution_clock Clock;

using namespace Eigen;
using namespace sensor_msgs;
using namespace std;
using namespace message_filters;

//Publisher
ros::Publisher pub_img_out;   // publish image mask
ros::Publisher time_pub;      // tiempo de ejecucion
ros::Publisher panel_LinesFeatures_pub; // validacion de color

// topics a suscribirse del nodo
std::string rgb_Topic   = "/camera/color/image_raw";
std::string depth_Topic = "/camera/aligned_depth_to_color/image_raw";



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

float hsv_v = 200;
float hsv_s = 100;

// Variables del filtrode kalman

cv::Vec4f lc_l, lc_r, l_pe_l, l_pe_r, lp_l, lp_r; // Variables linea anterior l_pe, linea actual(lc), linea actual derivada (lp) 

// Inicialización de y matrices
Eigen::MatrixXd A(8, 8), H(4, 8), Q(8, 8), R(4, 4), K(8, 4), P_left(8,8), P_right(8,8);;
Eigen::MatrixXd vec_lp_l(8,1), vec_lp_r(8,1);
bool inicio_kalman = true;

std::tuple<cv::Vec4f, Eigen::MatrixXd> kalman_filter( Eigen::VectorXd l_hat, float T, Eigen::MatrixXd P){
  Eigen::MatrixXd curr_state (4,1); // estados acutales
  curr_state << l_hat[0], l_hat[1], l_hat[2], l_hat[3];

  A << 1, 0, 0, 0, T, 0, 0, 0
      ,0, 1, 0, 0, 0, T, 0, 0
      ,0, 0, 1, 0, 0, 0, T, 0
      ,0, 0, 0, 1, 0, 0, 0, T
      ,0, 0, 0, 0, 1, 0, 0, 0
      ,0, 0, 0, 0, 0, 1, 0, 0
      ,0, 0, 0, 0, 0, 0, 1, 0
      ,0, 0, 0, 0, 0, 0, 0, 1;

  H << 1, 0, 0, 0, 0, 0, 0, 0
      ,0, 1, 0, 0, 0, 0, 0, 0
      ,0, 0, 1, 0, 0, 0, 0, 0
      ,0, 0, 0, 1, 0, 0, 0, 0;

  Q << 1, 0, 0, 0, 0, 0, 0, 0
      ,0, 1, 0, 0, 0, 0, 0, 0
      ,0, 0, 1, 0, 0, 0, 0, 0
      ,0, 0, 0, 1, 0, 0, 0, 0
      ,0, 0, 0, 0, 1, 0, 0, 0
      ,0, 0, 0, 0, 0, 1, 0, 0
      ,0, 0, 0, 0, 0, 0, 1, 0
      ,0, 0, 0, 0, 0, 0, 0, 1;

  R << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;

  // Predicción del estado siguiente
  l_hat = A * l_hat;

  P = A * P * A.transpose() + 1.0* Q;
  Eigen::MatrixXd var_m(4,4) ;
  var_m = H * P * H.transpose() + 100.0 *R;
  // Actualización del estado y la covarianza
  K = P * H.transpose() * var_m.inverse();

  l_hat = l_hat + K * (curr_state - H * l_hat);
  P = (Eigen::MatrixXd::Identity(8, 8) - K * H) * P;

  // envio los estados filtrados en forma de linea
  cv::Vec4f filterline;
  filterline[0] = l_hat[0];
  filterline[1] = l_hat[1];
  filterline[2] = l_hat[2];
  filterline[3] = l_hat[3];

return std::make_tuple(filterline,P);
}

// void callback(const CompressedImageConstPtr& in_rgb, const CompressedImageConstPtr& in_depth)
void callback(const ImageConstPtr& in_rgb, const ImageConstPtr& in_depth)
{
  auto t1 = Clock::now();
  ros::Time start_time = ros::Time::now();

  cv_bridge::CvImagePtr cv_rgbCam, cv_depthCam;
  try
  {
    cv_rgbCam =   cv_bridge::toCvCopy(in_rgb, sensor_msgs::image_encodings::BGR8);  // imagen de profundida de la camara 
    cv_depthCam = cv_bridge::toCvCopy(in_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  cv::Mat rgb_image    =    cv_rgbCam->image; 
  // Obtener las dimensiones de la imagen
  int imageHeight = rgb_image.rows; // rows
  int imageWidth  = rgb_image.cols; // cols


  /////////////////////////////////////////////////// Inicio  de Filtrado de la imagen a COLOR //////////////////////////////////////////
 
  // Definir el rango de colores blanco en HSV
  cv::Scalar lowerWhite;
  cv::Scalar upperWhite;

  if (real_sim) // bandera para escoger entre la simulacion y el real (real == True) (sim == False)
  {
    lowerWhite = cv::Scalar(0, 0, hsv_v);  // Umbral inferior para blanco
    upperWhite = cv::Scalar(180, hsv_s, 255);  // Umbral superior para blanco
  }
  else
  {
    lowerWhite = cv::Scalar(0, 0, 200);  // Umbral inferior para blanco
    upperWhite = cv::Scalar(180, 30, 255);  // Umbral superior para blanco
  }

  cv::Mat hsvImage;
  cv::cvtColor(rgb_image, hsvImage, cv::COLOR_BGR2HSV);

  // Binarizar la imagen utilizando el rango de colores blanco
  cv::Mat binaryImage;
  cv::inRange(hsvImage, lowerWhite, upperWhite, binaryImage);

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

  ////////////////////////////////////////////////////////// Fin del filtrado de la imagen a color //////////////////////////////////////////////////////
  
  // variables para detectar lineas con otro algoritmo
  std::vector<cv::Point>  contour_left;
  std::vector<cv::Point> contour_right;

  // Recorrer la imagen por filas y columnas
  for (int i = 100; i < imageHeight-100; i++) { // filas
      bool flag_left = false;
      bool flag_rigth = false;
      for (int j = 1; j < imageWidth/2; j++) { // columnas
          // Obtener el valor del pixel en (i, j)

          int pixel_left  = static_cast<int>(gray_image_filter.at<uchar>(i, j));
          int pixel_right = static_cast<int>(gray_image_filter.at<uchar>(i, imageWidth-j));

          if (pixel_left==255 && flag_left == false){
            contour_left.push_back(cv::Point(j, i));
            flag_left = true;
          }
          if (pixel_right==255 && flag_rigth == false){
            contour_right.push_back(cv::Point(imageWidth-j, i));
            flag_rigth = true;
          }
          if (flag_rigth && flag_left) // al ya tener las dos banderas en true se rompe el lazo for
            break;
          
      }
  }

  cv::Mat mono_resultImage = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);

  cv::Mat outputImage_points = cv::Mat::zeros(binaryImage.size(), CV_8UC3);

 
  //cv::drawContours(outputImage_points, std::vector<std::vector<cv::Point>>{contour_left}, 0, cv::Scalar(0, 255, 0), 1);
  for (const auto& point : contour_left) 
        cv::circle(outputImage_points, cv::Point(point.x, point.y), 1, cv::Scalar(255,255,255), -1);
  for (const auto& point : contour_right) 
        cv::circle(outputImage_points, cv::Point(point.x, point.y), 1, cv::Scalar(255,255,255), -1);

  //cv::drawContours(outputImage_points, std::vector<std::vector<cv::Point>>{contour_right}, 0, cv::Scalar(0, 0, 255), 1);

  /////////////////////////////////////////////////// Inicio  de Filtrado de la imagen depth //////////////////////////////////////////
  cv::Mat depthImage = cv_depthCam->image;
  cv::Mat depthValues;
  depthImage.convertTo(depthValues, CV_16U);


  // llamar a la función y guardar los resultados en dos objetos std::vector<cv::Point>
  auto [puntos_1, puntos_2] = processDepth(depthValues);


  contour_left.insert(contour_left.end(), puntos_1.begin(), puntos_1.end());
  contour_right.insert(contour_right.end(), puntos_2.begin(), puntos_2.end());

  for (const auto& point : puntos_1) 
        cv::circle(outputImage_points, cv::Point(point.x, point.y), 1, cv::Scalar(255,255,255), -1);
  for (const auto& point : puntos_2) 
        cv::circle(outputImage_points, cv::Point(point.x, point.y), 1, cv::Scalar(255,255,255), -1);

  std::vector<std::vector<cv::Point>> contours_2;
  contours_2.push_back(contour_left);  
  contours_2.push_back(contour_right);

  /////////////////////////////////////////////////// Fin de flitrado de imagen depth /////////////////////////////////////////////////
  ///////////////////////////////////////////////////// Inicio del filtro de kalman/////////////////////////////
  /*Al ya tener todos los puntos de los bordes del panel tanto de los canales rgb como del canal depth realziamos un filtro de kalman
  de la linea que se ajusta a los dos conjuntos de puntos detectados.*/ 
  
  auto t10= Clock::now();
  // Tiempo de muestreo
  float T = std::chrono::duration_cast<std::chrono::milliseconds>(t10-t1).count();
   T = T/1000.0;
  //std::cout << T << std::endl;


  std::vector<cv::Vec4f> linesap;
  int cont_line = 0;
  for (const auto& contour : contours_2) {

      // Ajustar una línea a los contornos
      if (contour.size()==0)
        break;
      cv::Vec4f line;
      cv::fitLine(contour, line, cv::DIST_L1, 0, 0.01, 0.01);

      float vx = line[0];
      float vy = line[1];
      float x  = line[2];
      float y  = line[3];



      if (inicio_kalman ){ // condicion para inicialiszar las varialbes del filtro de kalman
        lc_l = line;
        lc_r = line;

        vec_lp_l << lc_l[0], lc_l[1], lc_l[2], lc_l[3], 0, 0, 0, 0; // estados iniciales
        vec_lp_r << lc_r[0], lc_r[1], lc_r[2], lc_r[3], 0, 0, 0, 0; // estados iniciales

        P_left << 1, 0, 0, 0, 0, 0, 0, 0
                 ,0, 1, 0, 0, 0, 0, 0, 0
                 ,0, 0, 1, 0, 0, 0, 0, 0
                 ,0, 0, 0, 1, 0, 0, 0, 0
                 ,0, 0, 0, 0, 1, 0, 0, 0
                 ,0, 0, 0, 0, 0, 1, 0, 0
                 ,0, 0, 0, 0, 0, 0, 1, 0
                 ,0, 0, 0, 0, 0, 0, 0, 1; 

        P_right = P_left;
        inicio_kalman = false;

       }
      else{        
        if (cont_line == 0){
          lc_l = line;
          lp_l = (1/T)* (lc_l- l_pe_l);          
          vec_lp_l << lc_l[0], lc_l[1], lc_l[2], lc_l[3], lp_l[0], lp_l[1], lp_l[2], lp_l[3];
        }
        else{
          lc_r = line;
          lp_r = (1/T)* (lc_r- l_pe_r);          
          vec_lp_r << lc_r[0], lc_r[1], lc_r[2], lc_r[3], lp_r[0], lp_r[1], lp_r[2], lp_r[3];
        }
      }

      l_pe_l = lc_l;
      l_pe_r = lc_r;

      cv::Vec4f line_kalman_left(0,0,0,0);
      cv::Vec4f line_kalman_right(0,0,0,0);
      
      Eigen::MatrixXd outputP_left(8,8);
      Eigen::MatrixXd outputP_right(8,8);


      if (cont_line == 0){
        std::tie(line_kalman_left, outputP_left)   = kalman_filter(vec_lp_l, T, P_left);
        P_left  = outputP_left;

       //Puntos de la linea izquierda filtrada 
        cv::Point pt1_kalma_left(line_kalman_left[2] - 1000 * line_kalman_left[0], line_kalman_left[3] - 1000 * line_kalman_left[1]);
        cv::Point pt2_kalma_left(line_kalman_left[2] + 1000 * line_kalman_left[0], line_kalman_left[3] + 1000 * line_kalman_left[1]); 
        cv::line(outputImage_points, pt1_kalma_left, pt2_kalma_left, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
       }
      else{
        std::tie(line_kalman_right, outputP_right) = kalman_filter(vec_lp_r, T, P_right);
        P_right = outputP_right;
        //Puntos de la linea izquierda filtrada 
        cv::Point pt1_kalma_right(line_kalman_right[2] - 1000 * line_kalman_right[0], line_kalman_right[3] - 1000 * line_kalman_right[1]);
        cv::Point pt2_kalma_right(line_kalman_right[2] + 1000 * line_kalman_right[0], line_kalman_right[3] + 1000 * line_kalman_right[1]); 
        cv::line(outputImage_points, pt1_kalma_right, pt2_kalma_right, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

      }
     
      //Puntos de la linea sin filtrar
      cv::Point pt1_out_lin(x - 1000 * vx, y - 1000 * vy);
      cv::Point pt2_out_lin(x + 1000 * vx, y + 1000 * vy); 


      float angle = std::atan2(pt1_out_lin.y - pt2_out_lin.y, pt1_out_lin.x - pt2_out_lin.x) * 180 / CV_PI;

     // if (std::abs(angle) < 135 && std::abs(angle) > 45 ) {
        cv::line(mono_resultImage, pt1_out_lin, pt2_out_lin, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        cv::line(outputImage_points, pt1_out_lin, pt2_out_lin, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
      //}
 

      //  cv::line(mono_resultImage, pt1, pt2, cv::Scalar(255, 255, 255), 2);
      cont_line ++;
      
   }

  ////////////////////////////////////////////////////////////
   


  // A << 1, 0,  t, 0,
  //       0, 1, 0, 0,
  //       0, 0, 1, t,
  //       0, 0, 0, 1;

  // H << 1, 0, 0, 0,
  //       0, 1, 0, 0;

  // Q = Eigen::MatrixXd::Identity(4, 4); // Covarianza del proceso

  // R << 100, 0,
  //       0, 100; // Covarianza de la medición

  // // Predicción del estado siguiente

  // x_hat = A * x_hat;
  // P = A * P * A.transpose() + Q;

  // // Actualización del estado y la covarianza
  // K = P * H.transpose() / (H * P * H.transpose() + R);

  // m_actual.push_back(coefficients(0));
  // b_actual.push_back(coefficients(1));

  // if (k > 0) {
  //     m_p.push_back((m_actual[k] - m_anterior[k]) / T);
  //     b_p.push_back((b_actual[k] - b_anterior[k]) / T);
  // } else {
  //     m_p.push_back(0);
  //     b_p.push_back(0);
  // }

  // x_hat = x_hat + K * (Eigen::VectorXd(2) << m_actual[k], b_actual[k]).finished() - H * x_hat;
  // P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;

  // // Almacenar las coordenadas filtradas
  // filtered_m.push_back(x_hat(0));
  // filtered_b.push_back(x_hat(1));

  // m_anterior[k] = m_actual[k];
  // b_anterior[k] = b_actual[k];




  ////////////////////////////////////////////////////////7 fin filtro kalman      



  
  ros::Time end_time = ros::Time::now();  
  // Calcular la diferencia de tiempo
  ros::Duration delay_ros = end_time - start_time;
  
  sensor_msgs::ImagePtr image_msg_mask;
  image_msg_mask = cv_bridge::CvImage(std_msgs::Header(), "mono8", mono_resultImage).toImageMsg();
  image_msg_mask->header = in_rgb->header;
  image_msg_mask->header.stamp = in_rgb->header.stamp + delay_ros;
  pub_img_out.publish(image_msg_mask);  

  sensor_msgs::ImagePtr image_msg_rgb;
  image_msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage_points).toImageMsg();
  image_msg_rgb->header = in_rgb->header;
  image_msg_rgb->header.stamp = in_rgb->header.stamp + delay_ros;
  panel_LinesFeatures_pub.publish(image_msg_rgb);  

  // sensor_msgs::ImagePtr image_msg_hsv;
  // image_msg_hsv = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_image_filter).toImageMsg();
  // image_msg_hsv->header = in_rgb->header;
  // image_msg_hsv->header.stamp = in_rgb->header.stamp + delay_ros;
  // panel_hsvfilter.publish(image_msg_hsv);  

  // auto t10= Clock::now();
  // std::cout<<"time total (ms): "<<std::chrono::duration_cast<std::chrono::nanoseconds>(t10-t1).count()/1000000.0<<std::endl;
  //   // mensaje de tiempo de ejecucion
  // vision_matrice100::DurationStamped time_msg;  // Crear una instancia del mensaje
  // time_msg.header.stamp = in_rgb->header.stamp + delay_ros;  // Asignar la marca de tiempo actual al encabezado
  // time_msg.data = delay_ros;  // Asignar el valor a publicar al campo 'data' del mensaje

  // time_pub.publish(time_msg);  // Publicar el mensaje
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "panelFeatures_kalman");
  ros::NodeHandle nh;  

  // parametros para imagen a color

  nh.getParam("/rho", rho);
  nh.getParam("/theta", theta);
  nh.getParam("/threshold", threshold);
  nh.getParam("/minLineLen", minLineLen);
  nh.getParam("/maxLineGap", maxLineGap);
  nh.getParam("/minCan", minCan);
  nh.getParam("/maxCan", maxCan);
  nh.getParam("/area_filter", area_filter); 
  nh.getParam("/real_sim",real_sim);
  nh.getParam("/hsv_v", hsv_v); 
  nh.getParam("/hsv_s",hsv_s);
 
  // topics de la imagen 
  nh.getParam("/rgb_Topic", rgb_Topic);
  nh.getParam("/depth_Topic", depth_Topic);

  // message_filters::Subscriber<CompressedImage>  rgb_sub(nh, rgb_Topic , 10);
  // message_filters::Subscriber<CompressedImage>  depth_sub(nh, depth_Topic, 10);
  // typedef sync_policies::ApproximateTime<CompressedImage, CompressedImage> MySyncPolicy;

  message_filters::Subscriber<Image>  rgb_sub(nh, rgb_Topic , 10);
  message_filters::Subscriber<Image>  depth_sub(nh, depth_Topic, 10);
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2 ));
  
  pub_img_out = nh.advertise<sensor_msgs::Image>("/panel/image/mask/kalman", 10);

  panel_LinesFeatures_pub = nh.advertise<sensor_msgs::Image>("/panel/image/points", 10);

  ros::spin();
}

