/*programa del control servovisual del dron, como entrada necesita una mascara de dos lineas paralelas que son los paneles solares
este programa envia las velocidades de salida para el MPC*/
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
#include <csignal>

#include "sensor_msgs/Joy.h"

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "vision_matrice100/DurationStamped.h"


typedef std::chrono::high_resolution_clock Clock;

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace nav_msgs;

//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

//Publisher
ros::Publisher featuresRGB_pub;   // features in RGB image
ros::Publisher featurespcl_pub;   // features in pcl
ros::Publisher path_pub;          // path coordinates 
ros::Publisher veldroneWorld_pub; // velocidades dron- munod
ros::Publisher veldrone_pub;      // velocidades dron
ros::Publisher time_pub;          // tiempo de ejecucion
ros::Publisher err_servo_pub;     // errores del servovisual

// topics a suscribirse del nodo
std::string imgTopic   = "/camera/color/image_raw";
std::string depthTopic = "/camera/aligned_depth_to_color/image_raw";
std::string odom_dron  = "/dji_sdk/odometry";
std::string vel_drone_world_topic = "/dji_sdk/visual_servoing/vel/drone_world";
std::string vel_drone_topic = "/dji_sdk/visual_servoing/vel/drone";

bool control_ok = true;

float rho_hough = 1;
float theta = CV_PI/180;
float threshold = 100;
float angle_desired = 0;
float vx_lineal = 0.2; // velocidad hacia adelante en espacios nulos (m/s)

double d_panel_d = 300.0;//  distancia al panel deseada en pixeles

ros::Time delay_ros;
 

// matrices de calibracion entre la camara y el dron
Eigen::MatrixXf tc(3,1); // translation matrix dron-camera
Eigen::MatrixXf rc(3,3); // rotation matrix dron-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix

Eigen::MatrixXf Spxd(8,1);   // 4 puntos x,y en pixeles que la deteccion de los paneles puntos acutales
Eigen::MatrixXf lambda(8,1);  // ganancias seleccionadas en el control

Eigen::MatrixXf q_vel(6,1)   ; // velocidades del control servovisual
Eigen::MatrixXf q_vel_dw(6,1); // velocidades del control servovisual

// inicializacion de errores
float err_r = 0; 
float err_theta = 0;
float r_aux = 0;

// imagen de salida
cv::Mat resultImage;

void signalHandler(int signum)
{
  ROS_INFO("\nInterrupt signal received. Shutting down...");
    
      // Velocidades del cuerpo
  geometry_msgs::Twist velCuerpo_msgs;
  velCuerpo_msgs.linear.x =  0;    // Velocidad lineal en el eje x
  velCuerpo_msgs.linear.y =  0;    // Velocidad lineal en el eje y
  velCuerpo_msgs.linear.z =  0;    // Velocidad lineal en el eje z
  velCuerpo_msgs.angular.x = 0;   // Velocidad angular en el eje x
  velCuerpo_msgs.angular.y = 0;   // Velocidad angular en el eje y
  velCuerpo_msgs.angular.z = 0;   // Velocidad angular en el eje z

  // std::cout<< "velocidades dron: "<<std::endl<<q_vel<<std::endl;

  veldrone_pub.publish(velCuerpo_msgs);

  // velocidades del dron con respecto al mundo
  geometry_msgs::Twist velCuerpoMundo_msgs;  
  velCuerpoMundo_msgs.linear.x =  0;    // Velocidad lineal en el eje x
  velCuerpoMundo_msgs.linear.y =  0;    // Velocidad lineal en el eje y
  velCuerpoMundo_msgs.linear.z =  0;    // Velocidad lineal en el eje z
  velCuerpoMundo_msgs.angular.x = 0;   // Velocidad angular en el eje x
  velCuerpoMundo_msgs.angular.y = 0;   // Velocidad angular en el eje y
  velCuerpoMundo_msgs.angular.z = 0;   // Velocidad angular en el eje z

  // std::cout<< "velocidades dron-mundo: "<<std::endl<<q_vel_dw<<std::endl;

  veldroneWorld_pub.publish(velCuerpoMundo_msgs);


  ros::shutdown();
  exit(0);
}


///////////////////////////////////////callback
void callback(const ImageConstPtr& in_mask, const OdometryConstPtr& odom_msg)
{
  ros::Time start_time = ros::Time::now();

  cv_bridge::CvImagePtr  cv_maskImg;
      try
      {
        cv_maskImg = cv_bridge::toCvCopy(in_mask, sensor_msgs::image_encodings::MONO8);  // imagen de profundida de la camara 
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

  
  //cv::Mat img_rgbCam    = cv_rgbCam->image;
  cv::Mat mask_img  = cv_maskImg->image;

  int imageWidth = in_mask->width;
  int imageHeight = in_mask->height;

  
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Vec4f> linesap;
  for (const auto& contour : contours) {
      // Ajustar una línea a los contornos
      cv::Vec4f line;
      cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);
      // Guardar la línea
      linesap.push_back(line); 
  }

  // Dibujar las líneas aproximadas en la imagen original
  
  cv::cvtColor(mask_img, resultImage, cv::COLOR_GRAY2BGR);
  float ang1 = 0 ,ang2 = 0, x11 = 0, x12 = 0, y11 =0 , y12 =0, x21 = 0, x22 = 0, y21 =0 , y22 =0;// vyl1 = 0, vyl2 = 0,  yl1 = 0, yl2 = 0;
  float ang_ori1 = 0, ang_ori2 = 0;
  bool flag_lin1 = false, flag_lin2 = false;
  for (const auto& line : linesap) {
      float vx = line[0];
      float vy = line[1];
      float x = line[2];
      float y = line[3];

      //Puntos de la linea

      cv::Point pt1(x - 1000 * vx, y - 1000 * vy);
      cv::Point pt2(x + 1000 * vx, y + 1000 * vy);   

      if (x < imageWidth/2) {
        cv::line(resultImage, pt1, pt2, cv::Scalar(0, 0, 255), 2);
        x11 = pt1.x; 
        y11 = pt1.y; 
        x12 = pt2.x; 
        y12 = pt2.y; 
        ang1 = std::atan2(y12-y11, x12-x11) * 180 / CV_PI;
        ang_ori1 = ang1;
       // vyl1 = vy;
       // yl1 =y;

        if (vy>0){
          ang1 = ang1-180;
          
        }
        ang_ori1 = ang1;
       // std::cout<<"Line1: "<<line[0]<<", "<<line[1]<<", "<<line[2]<<", "<<line[3]<<std::endl;  
        flag_lin1 = true;    
      }
      else{
        
        cv::line(resultImage, pt1, pt2, cv::Scalar(0, 255, 0), 2);
        x21 = pt1.x;
        y21 = pt1.y; 
        x22 = pt2.x;
        y22 = pt2.y; 
       // vyl2 = vy;
       // yl2 = y;

        ang2 = std::atan2(y22-y21, x22-x21) * 180 / CV_PI;

        if (vy>0){
          ang2 = ang2-180;
        }
        ang_ori2 = ang2;

      }
     // std::cout<<"Line2: "<<line[0]<<", "<<line[1]<<", "<<line[2]<<", "<<line[3]<<std::endl;
      flag_lin2 = true;    

  }

 // std::cout<<"vyl2: "<<vyl1<<" vyl2:"<<vyl2<<" yl1: "<<yl1<<" yl2:"<<yl2<<std::endl;

  int xc = (x12+x11+x22+x21)/4;
  int yc = int(imageHeight/2);
  
  bool flag = flag_lin1 && flag_lin2;
  flag_lin1 = false;
  flag_lin2 = false;

  if (flag == false)
    xc = 0;
  
  float ang_t;
  if(ang1==0 || ang2==0 ){
    ang_t = 0;
    ang_ori1 = 0;
    ang_ori2 = 0;

  }
  else{
    ang_t = 90+(ang1+ang2)/2;
    ang_ori1 =  ang_ori1*CV_PI/180.0;
    ang_ori2 =  ang_ori2*CV_PI/180.0;
  }

  ang_t = ang_t*CV_PI/180.0;  


  // cv::Mat depthRoi;
  // // Obtener la región de interés en la imagen de profundidad
  // if (yc != 0 && xc != 0)
  //   depthRoi = depth_img(cv::Range(yc-100, yc+100), cv::Range(xc-2, xc+2));
  // else
  //   depthRoi = depth_img(cv::Range(imageHeight/2 - 10 , imageHeight/2 + 10 ), cv::Range(imageWidth/2 - 10, imageWidth/2+10 ));

  // // Calcular la profundidad promedio en el recuadro
  // double depthSum = cv::sum(depthRoi)[0];
  // double depthMean = (depthSum / (depthRoi.rows * depthRoi.cols))/1000.0;

  // std::cout<<"Rows: "<<depthRoi.rows<< "Cols: "<<depthRoi.cols<<std::endl;


  // // Obtener los valores de profundidad dentro del recuadro
  // std::vector<double> depthValues;

  // for (int ic=0; ic< depthRoi.rows; ic++){
  //     for (int jc=0; jc<depthRoi.cols ; jc++){
  //       auto px = depthRoi.at<ushort>(ic,jc);
  //           depthValues.push_back(px);
  //       }
  //   }

  // // Calcular la mediana
  // std::sort(depthValues.begin(), depthValues.end());
  // int n = depthValues.size();

  // std::cout<<"tamanno"<<depthValues.size()<<std::endl;

  // double median;
  // if (n % 2 == 0)
  //     median = (depthValues[n / 2 - 1] + depthValues[n / 2]) / 2.0;
  // else
  //     median = depthValues[n / 2];
  // double pixel_factor = 1000000.0/median;

  double x_l = (x11 + x12)/2 ; // punto x de la linea izquierda en metros
  double x_r = (x21 + x22)/2 ; // punto x de la linea derecha en metros 
  double y_l = yc;            // punto y de la linea izquierda en metros
  double y_r = yc;            // punto y de la linea derecha en metros

  // altura deseada y actual sacadas por la dsitancia entre los puntos medios sacados del panel.
  // estas generan un error que van a espacios nulos y que s emultiplican por una ganancia k_k

  double d_panel_r = sqrt(pow((x_r-x_l),2)+pow((y_l-y_r),2)); //  distancia al panel real en pixeles
 
  
  double pixel_factor = (d_panel_r/d_panel_d)*100;

  //condicion impedir un pixel_factor no deseado
  // if (d_panel_r<200 || d_panel_r>500)
  //   xc =0; // cuando este xc es igual a cero las velocidades del controlador tambien van a cero

  // puntos verticales
  float pv1_tx = (xc + pixel_factor*cos(ang_t-CV_PI/2));
  float pv1_ty = (yc + pixel_factor*sin(ang_t-CV_PI/2));
  float pv2_tx = (xc - pixel_factor*cos(ang_t-CV_PI/2));
  float pv2_ty = (yc - pixel_factor*sin(ang_t-CV_PI/2));


  // Deteccion de lineas del medio 
    // punto medio 
  cv::Point center(int(imageWidth/2),int(imageHeight/2));
  // Aplica la transformada de Hough
  cv::Mat line_path_img = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
  cv::line(line_path_img, cv::Point(pv1_tx, pv1_ty), cv::Point(pv2_tx, pv2_ty), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);



  // std::vector<cv::Vec2f> lineas;
  // cv::HoughLines(line_path_img, lineas, rho_hough, theta, threshold);

  // double x0 = 0;
  // double y0 = 0;
  // float theta_hou =0;
  // for (size_t i = 0; i < lineas.size(); i++) {
  //     float rho = lineas[i][0];
  //     theta_hou = lineas[i][1];

  //     if (rho<0)
  //       theta_hou -=CV_PI;
  //     rho = abs(rho);

  //     double a = std::cos(theta_hou);
  //     double b = std::sin(theta_hou);

  //     x0 = a * (rho);
  //     y0 = b * (rho);  

  //     cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * a));
  //     cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * a));

  //     //cv::line(resultImage, pt1, pt2, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  //   //  cv::circle(resultImage, cv::Point(x0,y0), 3, cv::Scalar(255, 255, 0),-1); 
  //     //cv::line(resultImage, cv::Point(0,0), cv::Point(x0,y0), cv::Scalar(255,0,  255), 1, cv::LINE_AA);

  //    // cv::line(resultImage, center,cv::Point(x0,y0), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

  //     // std::cout<<"Lineas con Hough: "<<pt1<<", "<<pt2<<std::endl;
  //     // std::cout<<"rho: "<<rho<<std::endl;
  //     // std::cout<<"theta: "<<(theta_hou*180/CV_PI)<<std::endl;
  //     // std::cout<<"x0: "<<x0<<"y0: "<<y0<<std::endl;
  //      // solo va a detectar la primera linea, ya que solo hice una linea con dos puntos, no necesito aproximar mas rectas
  // }

  

  // // puntos del cuadrado 
  // cv::Point pt1_pl(p1_tx,p1_ty);
  // cv::Point pt2_pl(p2_tx,p2_ty);
  // cv::Point pt3_pl(p3_tx,p3_ty);
  // cv::Point pt4_pl(p4_tx,p4_ty);

  // std::cout<<"angt: "<<ang_t<<"ang1: "<<ang1<<"ang2: "<<ang2<<std::endl;
  // std::cout<<"x1: " <<pt1_pl<<" p1: " <<std::endl;
  // std::cout<<"x2: " <<pt2_pl<<" p2: " <<std::endl;
  // std::cout<<"x3: " <<pt3_pl<<" p3: " <<std::endl;
  // std::cout<<"x4: " <<pt4_pl<<" p4: " <<std::endl;

  // Dibujar puntos deseados // deberia sacar la inversa de los puntos deseados de spxd (cuando haya tiempo hacemos)
  // cv::Point pt1_d(414, 242);
  // cv::Point pt2_d(199, 237);
  // cv::Point pt3_d(418, 135);
  // cv::Point pt4_d(201, 129);

  cv::Point pv1(pv1_tx, pv1_ty);
  cv::Point pv2(pv2_tx, pv2_ty);

  cv::circle(resultImage, pv1, 3, cv::Scalar(0, 0, 255), 3);
  cv::circle(resultImage, pv2, 3, cv::Scalar(255, 0, 0),3);

  // calculo de angulo solo para verificar
  // float angle_pixel = std::atan2(pt2_pl.x-pt1_pl.x,pt2_pl.y-pt1_pl.y);
  // std::cout<<"angt: "<<ang_t<<" angle_pixel: "<<angle_pixel<<std::endl;
  
  // sistema de coordenas
  cv::circle(resultImage, center, 3, cv::Scalar(255, 0, 0), 3);
  cv::line(resultImage, center,cv::Point(center.x+40,center.y), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
  cv::line(resultImage, center,cv::Point(center.x,center.y+40), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  cv::line(resultImage, cv::Point(pv1_tx,pv1_ty),cv::Point(pv2_tx,pv2_ty), cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  /// Distancia perpendicular 



  // //===================================================== jacobiana imagen ================================================================

    float lx = Mc(0,0);// focal de la camarax
    float ly = Mc(1,1);// focal de la camaray
    float zr = 1.0;


  // cambio de sistema de referencias de los puntos del centro de imagen
  float cx = Mc(0,2);
  float cy = Mc(1,2);
  Eigen::MatrixXf Hc(4,4);    // matriz de traslaciones con el centro de la camara
  Hc <<  1,0,0,cx
        ,0,1,0,cy
        ,0,0,1,0
        ,0,0,0,1;

  Eigen::MatrixXf pin_Hc = Hc.inverse();

  //////  cambio de referencia del punto sacado con Hough a medidas con el centro de la iamgen
  Eigen::MatrixXf x_hough  (4,2);
  Eigen::MatrixXf x_chough (4,2);
  x_hough << pv1_tx, pv2_tx,
             pv1_ty, pv2_ty,
             0, 0,
             1, 1;
  x_chough = pin_Hc *   x_hough; // punto de hough  ya normalizado desde el centro de la iamgen  

  // std::cout<<"x_chough: "<<x_chough<<std::endl;
  // float ang_chough = std::atan2 (x_chough(1,0),x_chough(0,0));
  // std::cout<<"x_chough_angulo grados: "<<ang_chough*180/CV_PI<<std::endl;
  // std::cout<<"x_chough_angulo radianes: "<<ang_chough<<std::endl;
  // float dist_choug = sqrt(pow(x_chough(0,0),2)+pow(x_chough(1,0),2));
  // std::cout<<"x_chough_dist: "<<dist_choug<<std::endl;
  // std::cout<<"x_chough_dist_comp_X: "<<dist_choug*cos(ang_chough)<<std::endl;

  /////////////////////////////////////////////////////////////////////
  Eigen::MatrixXf Spx(4,1);   // 4 puntos x,y en pixeles que la deteccion de los paneles puntos acutales
  Spx(0,0) = x_chough(0,0);
  Spx(1,0) = x_chough(1,0);
  Spx(2,0) = x_chough(0,1);
  Spx(3,0) = x_chough(1,1);

  Eigen::MatrixXf J_img(4,6); // jacobiana de imagen de 4 puntos de los objetos (4*2 x 6) matriz de medidas 8x6
    J_img << -(lx/zr) ,0.0    ,Spx(0,0)/zr  , (Spx(0,0)*Spx(1,0))/lx          ,-((lx*lx+Spx(0,0)*Spx(0,0))/lx)  , Spx(1,0)
            ,0.0    ,(-ly/zr) ,Spx(1,0)/zr  , (ly*ly+ (Spx(1,0)*Spx(1,0)))/ly ,-(Spx(0,0)*Spx(1,0))/ly          ,-Spx(0,0)

            ,-(lx/zr) ,0.0    ,Spx(2,0)/zr  , (Spx(2,0)*Spx(3,0))/lx          ,-((lx*lx+Spx(2,0)*Spx(2,0))/lx)  , Spx(3,0)
            ,0.0    ,(-ly/zr) ,Spx(3,0)/zr  , (ly*ly+ (Spx(3,0)*Spx(3,0)))/ly ,-(Spx(2,0)*Spx(3,0))/ly          ,-Spx(2,0);

            // ,-(lx/zr) ,0.0    ,Spx(4,0)/zr  , (Spx(4,0)*Spx(5,0))/lx          ,-((lx*lx+Spx(4,0)*Spx(4,0))/lx)  , Spx(5,0)
            // ,0.0    ,(-ly/zr) ,Spx(5,0)/zr  , (ly*ly+ (Spx(5,0)*Spx(5,0)))/ly  ,-(Spx(4,0)*Spx(5,0))/ly         ,-Spx(4,0)

            // ,-(lx/zr) ,0.0    ,Spx(6,0)/zr  , (Spx(6,0)*Spx(7,0))/lx          ,-((lx*lx+Spx(6,0)*Spx(6,0))/lx)  , Spx(7,0)
            // ,0.0    ,(-ly/zr) ,Spx(7,0)/zr  , (ly*ly+ (Spx(7,0)*Spx(7,0)))/ly ,-(Spx(6,0)*Spx(7,0))/ly          ,-Spx(6,0);

  //==========================================================================================================================================
   
  
  /////////////////////// Matriz de transforamciones para servovisual en el dron simulado (La camara está en el origen)

    Eigen::MatrixXf Tdc(3,1); // elemtnos de traslacion de la matriz RTcr
    Eigen::MatrixXf Rdc(3,3); // elementos de rotacion de la matriz RTcr
    Tdc << tc(0,0) ,tc(1,0) ,tc(2,0);
    Rdc << rc(0,0) ,rc(0,1) ,rc(0,2)
          ,rc(1,0) ,rc(1,1) ,rc(1,2) 
          ,rc(2,0) ,rc(2,1) ,rc(2,2);
            
    Eigen::MatrixXf ntc(3,3); //skew symmetric matrix traslation ntc = [0 -z y; z 0 -x; -y x 0] uso la traslacion de RTcb 

    ntc <<  0         ,-Tdc(2,0) , Tdc(1,0)
          , Tdc(2,0) , 0         ,-Tdc(0,0)
          ,-Tdc(1,0) , Tdc(0,0)  , 0      ;
          
    Eigen::MatrixXf nR_Tc(6,6);
    nR_Tc = Rdc.cwiseProduct(ntc); // Rdc x tdc

    Eigen::MatrixXf T_n(6,6);
    T_n << Rdc(0,0) ,Rdc(0,1) ,Rdc(0,2) ,nR_Tc(0,0) ,nR_Tc(0,1) ,nR_Tc(0,2) 
          ,Rdc(1,0) ,Rdc(1,1) ,Rdc(1,2) ,nR_Tc(1,0) ,nR_Tc(1,1) ,nR_Tc(1,2)
          ,Rdc(2,0) ,Rdc(2,1) ,Rdc(2,2) ,nR_Tc(2,0) ,nR_Tc(2,1) ,nR_Tc(2,2)
          ,0.0      ,0.0      ,0.0      ,Rdc(0,0)   ,Rdc(0,1)   ,Rdc(0,2)
          ,0.0      ,0.0      ,0.0      ,Rdc(1,0)   ,Rdc(1,1)   ,Rdc(1,2) 
          ,0.0      ,0.0      ,0.0      ,Rdc(2,0)   ,Rdc(2,1)   ,Rdc(2,2);
    
    Eigen::MatrixXf Jcam_robot(4,6);
    Jcam_robot = J_img * T_n;

    //Eigen::MatrixXf pin_Jimg = J_img.completeOrthogonalDecomposition().pseudoInverse();
    //Eigen::MatrixXf pin_Jc_r = Jcam_robot.completeOrthogonalDecomposition().pseudoInverse();


    ///////////////////////////LEY DE CONTROL SERVO VISUAL

    //////////////////// Nueva jacobiana

    // Eigen::MatrixXf J_th_im(2,6);
    float v1 = x_chough(1,0);
    float v2 = x_chough(1,1);
    float u1 = x_chough(0,0);
    float u2 = x_chough(0,1);
    float m = (u2-u1)/(v2-v1);

    

    //float dist_vu = sqrt(pow((v2-v1),2)+pow((u2-u1),2));
    //float angle_lin = std::atan2(u2-u1,v2-v1);
    float ang_theta = std::atan2((v2-v1),-(u2-u1));
    float r = sin (ang_theta) * (u1-(m)*v1);
    // float dist_vu_compx= dist_vu*sin(angle_lin);

    /// jacobiano como el paper
    Eigen::MatrixXf J_raux(1,5);
    J_raux << -sin(ang_theta)*(v1/(v1 - v2) - 1), 
              -sin(ang_theta)*((u1 - u2)/(v1 - v2) - (v1*(u1 - u2))/pow((v1 - v2),2)),
               (v1*sin(ang_theta))/(v1 - v2),
              -(v1*sin(ang_theta)*(u1 - u2))/pow((v1 - v2),2),
               cos(ang_theta)*(u1 - (v1*(u1 - u2))/(v1 - v2));

    Eigen::MatrixXf  J_r (2,5);
    J_r << J_raux(0), J_raux(1), J_raux(2), J_raux(3), J_raux(4),
                  0,          0,         0,         0,         1;

    Eigen::MatrixXf J_theta(5,4);
    Eigen::MatrixXf J_theta_aux(1,4);

    J_theta_aux << (v1-v2),-(u1-u2),-(v1-v2),(u1-u2);
    J_theta_aux = 1/(pow((v2-v1),2)+pow((u2-u1),2)) * J_theta_aux;

    J_theta << 1, 0, 0, 0, 
               0, 1, 0, 0, 
               0, 0, 1, 0,
               0, 0, 0, 1,
               J_theta_aux(0), J_theta_aux(1), J_theta_aux(2), J_theta_aux(3);
    

    // std::cout<<"***Jacobiana theta***"<<std::endl;
    // std::cout<<J_theta<<std::endl;
    // std::cout<<"***Jacobiana r***"<<std::endl;
    // std::cout<<J_r<<std::endl;       


    // std::cout<<"***Linea Amarilla***"<<std::endl;
    // std::cout<<"dist_vu: "<<dist_vu<<std::endl;
    // std::cout<<"angle_lin: "<<angle_lin*180/CV_PI<<std::endl;    

    // std::cout<<"dist_vu_compx: "<<dist_vu_compx<<std::endl;

    // std::cout<<"r: "<<r<<std::endl;
    // std::cout<<"ang_theta: "<<ang_theta*180/CV_PI<<std::endl;
    // std::cout<<"******************"<<std::endl;

    
    /////////////// jacobiana de salida
    Eigen::MatrixXf J_sal(2,6);
    J_sal = J_r * J_theta * Jcam_robot;

    Eigen::MatrixXf r_theta(2,1);

    err_r = 0.0 - r;
    err_theta = angle_desired - ang_theta  ;
    
    r_theta << lambda(0,0)* (err_r), 
               lambda(1,0)* (err_theta);

    r_aux = (err_r)/(imageWidth/2);
    //float r_aux = tanh((err_r)/lambda(5,0));

    Eigen::MatrixXf r_th_norm(2,1);
    r_th_norm << lambda(0,0)* r_aux, 
                 lambda(1,0)* (err_theta );

    Eigen::MatrixXf r_th_wo_gain(2,1);
    r_th_wo_gain << r_aux, 
                    err_theta;
      

    // std::cout<<"Error norma: "<<r_th_norm.norm()<<std::endl;
    // std::cout<<"Errores: "<<r_th_norm<<std::endl;             
   

    // matriz para espacio nulo
    Eigen::MatrixXf gain_null(6,6);
    gain_null << 1 ,0 ,0 ,0 ,0 ,0
                ,0 ,1 ,0 ,0 ,0 ,0
                ,0 ,0 ,1 ,0 ,0 ,0
                ,0 ,0 ,0 ,lambda(3,0) ,0 ,0
                ,0 ,0 ,0 ,0 ,lambda(4,0) ,0
                ,0 ,0 ,0 ,0 ,0 ,1;
                
    gain_null = lambda(2,0)*gain_null ;

    Eigen::MatrixXf v_null(6,1);
    v_null << vx_lineal/(1+r_th_wo_gain.norm()),
              0,
              lambda(6,0)*(d_panel_r-d_panel_d),
              0,
              0,
              0;    
    
    Eigen::MatrixXf Iden(6,6);
    Iden << 1 ,0 ,0 ,0 ,0 ,0
                ,0 ,1 ,0 ,0 ,0 ,0
                ,0 ,0 ,1 ,0 ,0 ,0
                ,0 ,0 ,0 ,1 ,0 ,0
                ,0 ,0 ,0 ,0 ,1 ,0
                ,0 ,0 ,0 ,0 ,0 ,1;
 
    Eigen::MatrixXf J_sal_trans = J_sal.transpose();

    Eigen::MatrixXf in_J_aux = J_sal * gain_null.inverse()* J_sal_trans;
    Eigen::MatrixXf in_J_sal = in_J_aux.inverse();
    in_J_sal = gain_null.inverse()*J_sal_trans * in_J_sal;


    q_vel = in_J_sal * r_th_norm + (Iden - in_J_sal* J_sal ) * v_null ;  //calculo de velocidades de salida que van al robot solo para el angulo theta

    // //// nueva jacobiana aumentada
    // Eigen::MatrixXf J_aux(2,2);
    // J_aux << sin(angle_lin) , dist_vu*cos(angle_lin),
    //           0,                1;

    // //sin(angle_lin) , dist_vu*cos(angle_lin),

    // Eigen::MatrixXf J_aux2(2,4);

    // J_aux2 <<     (u1-u2)/dist_vu, (v1 -v2)/dist_vu, -(u1-u2)/dist_vu, -(v1 -v2)/dist_vu,
    //               J_theta(0,0),    J_theta(0,1),    J_theta(0,2),    J_theta(0,3);


    // J_th_im = J_aux * J_aux2 * Jcam_robot;

   // Eigen::MatrixXf pin_J_th_im = J_th_im.completeOrthogonalDecomposition().pseudoInverse();

    // Eigen::MatrixXf theta_p(2,1);
    
    // theta_p << lambda(0,0)* (dist_vu_compx - 0.0), 
    //            lambda(1,0)* (angle_lin - 0.0);
        
  //////////// Envio de resultados de velocidades al topic de odometria 

  if (!control_ok || xc ==0 )
    q_vel << 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0 ;
 
  ////////////// conversion velocidades dron mundo
  
  double x_q = odom_msg->pose.pose.orientation.x;
  double y_q = odom_msg->pose.pose.orientation.y;
  double z_q = odom_msg->pose.pose.orientation.z;
  double w_q = odom_msg->pose.pose.orientation.w;


  Eigen::Quaterniond quaternion(w_q, x_q, y_q, z_q); // w, x, y, z
  Eigen::Matrix3d RdW = quaternion.toRotationMatrix();

  Eigen::MatrixXf d_W(6,6);

  d_W << RdW(0,0) ,RdW(0,1) ,RdW(0,2) ,0.0      ,0.0      ,0.0
        ,RdW(1,0) ,RdW(1,1) ,RdW(1,2) ,0.0      ,0.0      ,0.0
        ,RdW(2,0) ,RdW(2,1) ,RdW(2,2) ,0.0      ,0.0      ,0.0
        ,0.0      ,0.0      ,0.0      ,RdW(0,0)   ,RdW(0,1)   ,RdW(0,2)
        ,0.0      ,0.0      ,0.0      ,RdW(1,0)   ,RdW(1,1)   ,RdW(1,2) 
        ,0.0      ,0.0      ,0.0      ,RdW(2,0)   ,RdW(2,1)   ,RdW(2,2);

  q_vel_dw = d_W * q_vel;

 ros::Time end_time = ros::Time::now();  
  // Calcular la diferencia de tiempo
 ros::Time time_mask = in_mask->header.stamp;
 ros::Duration delay_pro = end_time - start_time; 
 delay_ros = time_mask + delay_pro;

}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "panelFeatures");
  ros::NodeHandle nh;  
  
  /// Load Parameters
  std::cout<<"Features Node initialized... :)";

  nh.getParam("/imgTopic", imgTopic);
  //nh.getParam("/depthTopic", depthTopic);
  nh.getParam("/odom_dron", odom_dron);

  nh.getParam("/rho", rho_hough);
  nh.getParam("/theta", theta);
  nh.getParam("/threshold", threshold);

  nh.getParam("/control_ok", control_ok);
  nh.getParam("/angle_desired", angle_desired);

  nh.getParam("/publish_vel_drone_topic", vel_drone_topic);
  nh.getParam("/publish_vel_drone_world_topic", vel_drone_world_topic);
  nh.getParam("/vx_lineal",vx_lineal);
  nh.getParam("/d_panel_d",d_panel_d);

    
  XmlRpc::XmlRpcValue param;

  nh.getParam("/vision_params/tc", param);
  tc <<  (double)param[0]
         ,(double)param[1]
         ,(double)param[2];

  nh.getParam("/vision_params/rc", param);

  rc <<  (double)param[0] ,(double)param[1] ,(double)param[2]
         ,(double)param[3] ,(double)param[4] ,(double)param[5]
         ,(double)param[6] ,(double)param[7] ,(double)param[8];

  nh.getParam("/vision_params/camera_matrix", param);

  Mc  <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3]
         ,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7]
         ,(double)param[8] ,(double)param[9] ,(double)param[10],(double)param[11];

  nh.getParam("/vision_params/spxd", param);
  // spxd data
  Spxd <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3] ,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7];
  //Spxd <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3];


  nh.getParam("/vision_params/lambda", param);
  // ganacias del control cineamtico
  lambda <<  (double)param[0], (double)param[1], (double)param[2], (double)param[3], (double)param[4], (double)param[5], (double)param[6], (double)param[7];

  // Registrar el manejador de señal para SIGINT
    signal(SIGINT, signalHandler);

  message_filters::Subscriber<Image>    depthCam_sub(nh, imgTopic, 5);
  message_filters::Subscriber<Odometry> odom_robot(nh, odom_dron, 5);

  typedef sync_policies::ApproximateTime<Image, Odometry> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), depthCam_sub, odom_robot);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  featuresRGB_pub = nh.advertise<sensor_msgs::Image>("/dji_sdk/visual_servoing/img_features", 10);
  veldrone_pub = nh.advertise<geometry_msgs::TwistStamped>(vel_drone_topic, 10);
  veldroneWorld_pub = nh.advertise<geometry_msgs::TwistStamped>(vel_drone_world_topic, 10);
  err_servo_pub = nh.advertise<geometry_msgs::TwistStamped>("/dji_sdk/visual_servoing/errores", 10);
  time_pub = nh.advertise<vision_matrice100::DurationStamped>("/dji_sdk/visual_servoing/runtime", 10);  

  ros::Rate loop_rate(20);

  while (ros::ok())
  {    
    ros::spinOnce();
    auto t1 = Clock::now();
    ros::Time duration = delay_ros;
    // mensaje de tiempo de ejecucion
    vision_matrice100::DurationStamped time_msg;  // Crear una instancia del mensaje
    time_msg.header.stamp =  duration;  // Asignar la marca de tiempo actual al encabezado
    time_msg.data = duration - ros::Time(0);  // Asignar el valor a publicar al campo 'data' del mensaje
    
    // Velocidades del cuerpo
    geometry_msgs::TwistStamped  velCuerpo_msgs;

    velCuerpo_msgs.header.frame_id = "base_link";
    velCuerpo_msgs.header.stamp = duration;

    velCuerpo_msgs.twist.linear.x =  q_vel(0,0);    // Velocidad lineal en el eje x
    velCuerpo_msgs.twist.linear.y =  q_vel(1,0);    // Velocidad lineal en el eje y
    velCuerpo_msgs.twist.linear.z =  q_vel(2,0);    // Velocidad lineal en el eje z
    velCuerpo_msgs.twist.angular.x = q_vel(3,0);   // Velocidad angular en el eje x
    velCuerpo_msgs.twist.angular.y = q_vel(4,0);   // Velocidad angular en el eje y
    velCuerpo_msgs.twist.angular.z = q_vel(5,0);   // Velocidad angular en el eje z

    // std::cout<< "velocidades dron: "<<std::endl<<q_vel<<std::endl;

    // velocidades del dron con respecto al mundo
    geometry_msgs::TwistStamped  velCuerpoMundo_msgs;  

    velCuerpoMundo_msgs.header.frame_id = "world";
    velCuerpoMundo_msgs.header.stamp =  duration;

    velCuerpoMundo_msgs.twist.linear.x =  q_vel_dw(0,0);    // Velocidad lineal en el eje x
    velCuerpoMundo_msgs.twist.linear.y =  q_vel_dw(1,0);    // Velocidad lineal en el eje y
    velCuerpoMundo_msgs.twist.linear.z =  q_vel_dw(2,0);    // Velocidad lineal en el eje z
    velCuerpoMundo_msgs.twist.angular.x = q_vel_dw(3,0);   // Velocidad angular en el eje x
    velCuerpoMundo_msgs.twist.angular.y = q_vel_dw(4,0);   // Velocidad angular en el eje y
    velCuerpoMundo_msgs.twist.angular.z = q_vel_dw(5,0);   // Velocidad angular en el eje z

    //veldroneWorld_pub.publish(velCuerpoMundo_msgs);

    // std::cout<< "velocidades dron-mundo: "<<std::endl<<q_vel_dw<<std::endl;

    // publicacion de los errores del servovisual en un topic tipo velocidad(solo es apra ver los errores)
    geometry_msgs::TwistStamped  err_servo_msg;  

    

    err_servo_msg.header.frame_id = "base_link";
    err_servo_msg.header.stamp =  duration ;

    err_servo_msg.twist.linear.x =  err_r;    // errores de r no normalizado (0.0 -r)
    err_servo_msg.twist.linear.y =  err_theta;    // errores de theta no normalizado (angulo_deseado th)
    err_servo_msg.twist.linear.z =  r_aux;  //  normalizado
    err_servo_msg.twist.angular.x = err_theta;  // errores de theta normalizado

    //err_servo_pub.publish(err_servo_msg);
      
    sensor_msgs::ImagePtr image_msg;
    image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resultImage).toImageMsg();  
    image_msg->header.stamp  = duration;
  
    //featuresRGB_pub.publish(image_msg);   


    time_pub.publish(time_msg);
    veldrone_pub.publish(velCuerpo_msgs);
    veldroneWorld_pub.publish(velCuerpoMundo_msgs);
    err_servo_pub.publish(err_servo_msg);
    featuresRGB_pub.publish(image_msg);    

    auto t2= Clock::now();
    std::cout<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()/1000000.0<<std::endl;
    loop_rate.sleep();
  }
  
  return 0 ;
  //ros::spin();

}
