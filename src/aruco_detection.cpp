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
#include <opencv2/aruco.hpp>
#include <iostream>
#include <chrono> 

// markers
#include <visualization_msgs/Marker.h>

#include "sensor_msgs/Joy.h"

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Twist.h"

typedef std::chrono::high_resolution_clock Clock;

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace nav_msgs;



//Publisher
ros::Publisher featuresRGB_pub;   // features in RGB image
//ros::Publisher featuresDepth_pub; // features in depth image
ros::Publisher featurespcl_pub;   // features in pcl
ros::Publisher path_pub;          // path coordinates 
ros::Publisher twist_pub;         // velocidades

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
bool control_ok = true;

// matrices de calibracion entre la camara y el dron
Eigen::MatrixXf tc(3,1); // translation matrix dron-camera
Eigen::MatrixXf rc(3,3); // rotation matrix dron-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix

Eigen::MatrixXf Spxd(4,1);   // 4 puntos x,y en pixeles que la deteccion de los paneles puntos acutales
Eigen::MatrixXf lambda(6,1);  // ganancias seleccionadas en el control

Eigen::MatrixXf q_vel(6,1) ; // velocidades del control servovisual
Eigen::MatrixXf q_vel_cam(6,1) ; // velocidades del control servovisual


///////////////////////////////////////callback
 void callback(const ImageConstPtr& in_RGB)
{
      
  auto t1 = Clock::now();

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

      // Detectar el marcador ArUco en la imagen
  cv::Mat image = cv_rgbCam->image;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  cv::aruco::detectMarkers(image, dictionary, corners, ids);



  if (ids.size() == 0)
      ROS_INFO("Marcador ArUco no detectado");
  else{

  cv::Mat camera_matrix (3,3,CV_32FC1);
  camera_matrix.at<float>(0, 0) = 204.0;
  camera_matrix.at<float>(0, 1) = 0.0;
  camera_matrix.at<float>(0, 2) = 318.0;
  camera_matrix.at<float>(1, 0) = 0.0;
  camera_matrix.at<float>(1, 1) = 204.0;
  camera_matrix.at<float>(1, 2) = 238.69;
  camera_matrix.at<float>(2, 0) = 0.0;
  camera_matrix.at<float>(2, 1) = 0.0;
  camera_matrix.at<float>(2, 2) = 1.0;

  cv::Mat distortion_coefficients (1,5,CV_32FC1);
		int k=0;
		for (int i=0;i<5;i++){		
			distortion_coefficients.at<float>(0, i) = 0;
			k = k+1;
		}


  std::vector< cv::Vec3d > rvec, tvec;
		if(ids.size() > 0) {
		  cv::aruco::estimatePoseSingleMarkers(corners, 0.185, camera_matrix, distortion_coefficients, rvec, tvec);
			cv::aruco::drawAxis(image, camera_matrix, distortion_coefficients, rvec[0], tvec[0], 0.185);
      //cv::aruco::drawDetectedMarkers(image, corners, ids);
		}


    // Imprimir coordenadas de las esquinas
    for (int i = 0; i < ids.size(); i++) {
        cout << "Marcador ID: " << ids[i] << endl;
        cout << "Esquinas: " << endl;
        for (int j = 0; j < 4; j++) {
            cout << "[" << corners[i][j].x << ", " << corners[i][j].y << "]" << endl;
        }
    }

    cv::Point pt1_pl(corners[0][0].x,corners[0][0].y);
    cv::Point pt2_pl(corners[0][1].x,corners[0][1].y);
    // cv::Point pt3_pl(corners[0][2].x,corners[0][2].y);
    // cv::Point pt4_pl(corners[0][3].x,corners[0][3].y);

    cv::circle(image, pt1_pl, 5, cv::Scalar(255, 0, 0), -1);
    cv::circle(image, pt2_pl, 5, cv::Scalar(0, 255, 0), -1);
    // cv::circle(image, pt3_pl, 5, cv::Scalar(0, 0, 255), -1);
    // cv::circle(image, pt4_pl, 5, cv::Scalar(255, 0, 255), -1);

    cv::Point pt1_d(255,283);
    cv::Point pt2_d(255,221);
    // cv::Point pt3_d(317,221);
    // cv::Point pt4_d(317,283);

    cv::circle(image, pt1_d, 6, cv::Scalar(255, 0, 0), 1);
    cv::circle(image, pt2_d, 6, cv::Scalar(0, 255, 0), 1);
    // cv::circle(image, pt3_d, 6, cv::Scalar(0, 0, 255), 1);
    // cv::circle(image, pt4_d, 6, cv::Scalar(255, 0, 255),1);

  uint xc = 1;

  // Publicar las coordenadas del marcador
  if (ids.size() > 0)
  {
    //cv::aruco::drawDetectedMarkers(image, corners, ids);
    ROS_INFO("Marcador ArUco detectado con ID %d en la posición (%f, %f)",
             ids[0], corners[0][0].x, corners[0][0].y);

    for (int i = 0; i<ids.size(); i++){
		  std::cout<<"Rotation of aruco number "<<i+1<<"/"<<ids.size()<<": "<<std::endl;
			std::cout<<rvec[i]<<std::endl;
			std::cout<<"Translation of aruco number "<<i+1<<"/"<<ids.size()<<": "<<std::endl;
			std::cout<<tvec[i]<<std::endl;
			std::cout<<""<<std::endl;
		}


   }
  else
  {
    ROS_INFO("Marcador ArUco no detectado");
     xc = 0;
  }

  // //===================================================== jacobiana imagen ================================================================

    float lx = Mc(0,0);// focal de la camarax
    float ly = Mc(1,1);// focal de la camaray
    float cx = Mc(0,2);
    float cy = Mc(1,2);
    float zr = sqrt(tvec[0][0]*tvec[0][0]+tvec[0][1]*tvec[0][1]+tvec[0][2]*tvec[0][2]);
    Eigen::MatrixXf Spx(4,1);   // 2 puntos x,y en pixeles que la deteccion de los paneles puntos acutales

    Spx(0,0) = pt1_pl.x;
    Spx(1,0) = pt1_pl.y;
    Spx(2,0) = pt2_pl.x;
    Spx(3,0) = pt2_pl.y;

    // Spx(4,0) = pt3_pl.x;
    // Spx(5,0) = pt3_pl.y;
    // Spx(6,0) = pt4_pl.x;
    // Spx(7,0) = pt4_pl.y;

  // matriz de traslaciones con el centro de la camara
  Eigen::MatrixXf Hc(4,4); 
  Hc << 1,0,0,cx
        ,0,1,0,cy
        ,0,0,1,0
        ,0,0,0,1;

  Eigen::MatrixXf pin_Hc = Hc.completeOrthogonalDecomposition().pseudoInverse();

  Eigen::MatrixXf x_c (4,2);
  // x_c<< Spx(0,0), Spx(2,0), Spx(4,0), Spx(6,0),
  //       Spx(1,0), Spx(3,0), Spx(5,0), Spx(7,0),
  //       0,        0,        0,        0,
  //       1,        1,        1,        1;

  x_c<< Spx(0,0), Spx(2,0), 
        Spx(1,0), Spx(3,0), 
        0,        0,        
        1,        1       ;



  Eigen::MatrixXf x_ct (4,2);
  x_ct = pin_Hc * x_c;

  for (int i = 0;i<4;i++){
    for (int j = 0;j<2;j++){
      std::cout<< x_ct(i,j)<<", ";
    }
     std::cout<<std::endl;
  }


  Spx(0,0) = x_ct(0,0);
  Spx(1,0) = x_ct(1,0);
  Spx(2,0) = x_ct(0,1);
  Spx(3,0) = x_ct(1,1);

  // Spx(4,0) = x_ct(0,2);
  // Spx(5,0) = x_ct(1,2);
  // Spx(6,0) = x_ct(0,3);
  // Spx(7,0) = x_ct(1,3);


    Eigen::MatrixXf J_img(4,6); // jacobiana de imagen de 2 puntos de los objetos (2*2 x 6) matriz de medidas 4x6
    J_img << -(lx/zr) ,0.0    ,Spx(0,0)/zr  , (Spx(0,0)*Spx(1,0))/lx     , -((lx*lx+Spx(0,0)*Spx(0,0))/lx),Spx(1,0)
            ,0.0    ,(-ly/zr) ,Spx(1,0)/zr  , (ly*ly+ (Spx(1,0)*Spx(1,0)))/ly  ,-(Spx(0,0)*Spx(1,0))/ly   ,-Spx(0,0)

            ,-(lx/zr) ,0.0    ,Spx(2,0)/zr  , (Spx(2,0)*Spx(3,0))/lx     ,-((lx*lx+Spx(2,0)*Spx(2,0))/lx) , Spx(3,0)
            ,0.0    ,(-ly/zr) ,Spx(3,0)/zr  , (ly*ly+ Spx(3,0)*Spx(3,0))/ly  ,-(Spx(2,0)*Spx(3,0))/ly     ,-Spx(2,0);

            // ,-(lx/zr) ,0.0    ,Spx(4,0)/zr  , (Spx(4,0)*Spx(5,0))/lx     ,-((lx*lx+Spx(4,0)*Spx(4,0))/lx) , Spx(5,0)
            // ,0.0    ,(-ly/zr) ,Spx(5,0)/zr  , (ly*ly+ Spx(5,0)*Spx(5,0))/ly  ,-(Spx(4,0)*Spx(5,0))/ly     ,-Spx(4,0)

            // ,-(lx/zr) ,0.0    ,Spx(6,0)/zr  , (Spx(6,0)*Spx(7,0))/lx     ,-((lx*lx+Spx(6,0)*Spx(6,0))/lx) , Spx(7,0)
            // ,0.0    ,(-ly/zr) ,Spx(7,0)/zr  , (ly*ly+ Spx(7,0)*Spx(7,0))/ly  ,-(Spx(6,0)*Spx(7,0))/ly     ,-Spx(6,0);


  /////////////////////// Matriz de transforamciones para servovisual en el dron simulado (La camara está en el origen)

    Eigen::MatrixXf Tdc(3,1); // elemtnos de traslacion de la matriz RTcr
    Eigen::MatrixXf Rdc(3,3); // elementos de rotacion de la matriz RTcr
    Tdc << tc(0,0) ,tc(1,0) ,tc(2,0);
    Rdc << rc(0,0) ,rc(0,1) ,rc(0,2)
          ,rc(1,0) ,rc(1,1) ,rc(1,2) 
          ,rc(2,0) ,rc(2,1) ,rc(2,2);
            
    //std::cout<<"Tcb:"<<Tcr<<std::endl;
    //std::cout<<"Rcb:"<<Rcr<<std::endl;

    Eigen::MatrixXf ntc(3,3); //skew symmetric matrix traslation ntc = [0 -z y; z 0 -x; -y x 0] uso la traslacion de RTcb 

    ntc <<  0         ,-Tdc(2,0) , Tdc(1,0)
          , Tdc(2,0) , 0         ,-Tdc(0,0)
          ,-Tdc(1,0) , Tdc(0,0)  , 0      ;
          
    Eigen::MatrixXf nR_Tc(6,6);
    nR_Tc = Rdc.cwiseProduct(ntc);

    Eigen::MatrixXf T_n(6,6);
    T_n << Rdc(0,0) ,Rdc(0,1) ,Rdc(0,2) ,nR_Tc(0,0) ,nR_Tc(0,1) ,nR_Tc(0,2) 
          ,Rdc(1,0) ,Rdc(1,1) ,Rdc(1,2) ,nR_Tc(1,0) ,nR_Tc(1,1) ,nR_Tc(1,2)
          ,Rdc(2,0) ,Rdc(2,1) ,Rdc(2,2) ,nR_Tc(2,0) ,nR_Tc(2,1) ,nR_Tc(2,2)
          ,0.0      ,0.0      ,0.0      ,Rdc(0,0)   ,Rdc(0,1)   ,Rdc(0,2)
          ,0.0      ,0.0      ,0.0      ,Rdc(1,0)   ,Rdc(1,1)   ,Rdc(1,2) 
          ,0.0      ,0.0      ,0.0      ,Rdc(2,0)   ,Rdc(2,1)   ,Rdc(2,2);
    
    Eigen::MatrixXf Jcam_robot(4,6);
    Jcam_robot = J_img * T_n;

    Eigen::MatrixXf pin_Jc_r = Jcam_robot.completeOrthogonalDecomposition().pseudoInverse();


    Eigen::MatrixXf Sptan(4,1);
    Sptan(0,0) = lambda(0)* tanh((zr/((lx+ly)/2))*(Spx(0,0)-Spxd(0,0))); 
    Sptan(1,0) = lambda(1)* tanh((zr/((lx+ly)/2))*(Spx(1,0)-Spxd(1,0))); 
    Sptan(2,0) = lambda(2)* tanh((zr/((lx+ly)/2))*(Spx(2,0)-Spxd(2,0))); 
    Sptan(3,0) = lambda(3)* tanh((zr/((lx+ly)/2))*(Spx(3,0)-Spxd(3,0))); 



    Eigen::MatrixXf Sperr(4,1);
    Sperr(0,0) = (Spx(0,0)-Spxd(0,0)); 
    Sperr(1,0) = (Spx(1,0)-Spxd(1,0)); 
    Sperr(2,0) = (Spx(2,0)-Spxd(2,0)); 
    Sperr(3,0) = (Spx(3,0)-Spxd(3,0)); 


    std::cout<<"Norma error: "<<(zr/((lx+ly)/2))*Sperr.norm()<<std::endl;



    //q_vel =  - pin_Jc_r * (Spx - Spxd);  //calculo de velocidades de salida que van al robot 
    q_vel =  - pin_Jc_r * (Sptan);  //calculo de velocidades de salida que van al robot 
    

    // q_vel << lambda(0,0)*q_vel(0,0), lambda(1,0)*q_vel(1,0), lambda(2,0)*q_vel(2,0),
    //          lambda(3,0)*q_vel(3,0), lambda(4,0)*q_vel(4,0),lambda(5,0)*q_vel(5,0); // ganancias apra velocidades de salida

    //////////// Envio de resultados de velocidades al topic de odometria 

  if (xc == 0)
    q_vel << 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0 ;

  geometry_msgs::Twist twist_msg;
  
  // Configurar las velocidades en el mensaje Twist
  twist_msg.linear.x =  q_vel(0,0);    // Velocidad lineal en el eje x
  twist_msg.linear.y =  q_vel(1,0);    // Velocidad lineal en el eje y
  twist_msg.linear.z =  q_vel(2,0);    // Velocidad lineal en el eje z
  twist_msg.angular.x = q_vel(3,0);   // Velocidad angular en el eje x
  twist_msg.angular.y = q_vel(4,0);   // Velocidad angular en el eje y
  twist_msg.angular.z = q_vel(5,0);   // Velocidad angular en el eje z

   if (control_ok)
   twist_pub.publish(twist_msg);



  int cols_img = in_RGB->width;
  int rows_img = in_RGB->height;

  cv::Mat imgGrayscale;        // grayscale of input image
  cv::Mat imgBlurred;            // intermediate blured image
  cv::Mat imgCanny;            // Canny edge image


  cv::cvtColor(img_rgbCam, imgGrayscale, CV_BGR2GRAY);        // convert to grayscale

  
  sensor_msgs::ImagePtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  image_msg->header.stamp     = in_RGB->header.stamp;
 
  featuresRGB_pub.publish(image_msg);
  auto t2= Clock::now();
  std::cout<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()/1000000.0<<std::endl;

  std::cout<<"Velocidades XYZ, PRY"<<std::endl;
  for(int i = 0;i<6;i++){
    std::cout<< q_vel(i,0)<<std::endl;
  }


  }
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "arucoDetection");
  ros::NodeHandle nh;  
  
  /// Load Parameters
  std::cout<<"Aruco_detection Node initialized... :)";

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/angle_vertical_lines", angLin);
  nh.getParam("/imgTopic", imgTopic);
  nh.getParam("/real_flag", real_flag);
  nh.getParam("/tfPCL", tf_pcl);
  nh.getParam("/tf_word_or_sensor", tf_wOs);
  nh.getParam("/control_ok", control_ok);
  
    
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
  Spxd <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3]; //,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7];

  nh.getParam("/vision_params/lambda", param);
  lambda <<  (double)param[0], (double)param[1], (double)param[2], (double)param[3], (double)param[4], (double)param[5];
 


  ros::Subscriber sub = nh.subscribe<Image>(imgTopic, 10, callback);

  featuresRGB_pub = nh.advertise<sensor_msgs::Image>("/imgFeatures", 10);
  twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
 
  ros::spin();

}