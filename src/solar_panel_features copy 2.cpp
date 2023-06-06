#include <ros/ros.h>
#include <limits>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
//#include <pcl/point_types.h>
//#include <pcl/range_image/range_image.h>
//#include <pcl/range_image/range_image_spherical.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/impl/point_types.hpp>
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
#include "geometry_msgs/Twist.h"

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
ros::Publisher twist_pub;         // velocidades

// topics a suscribirse del nodo
std::string imgTopic   = "/camera/color/image_raw";
std::string depthTopic = "/camera/aligned_depth_to_color/image_raw";
std::string odom_dron  = "/dji_sdk/odometry";
std::string tf_pcl     = "camera_link";
bool tf_wOs = true;  // true mundo,  false sensor
bool control_ok = true;

float minlen = 0.5;
float maxlen = 10.0;
float angLin = 10.0;

int minCan = 80;
int maxCan = 100;


float rho = 1;
float theta = CV_PI/180;
float threshold = 100;
float minlenli = 1;
float maxlenli = 10;
float angle_desired = 0;
 

// matrices de calibracion entre la camara y el dron
Eigen::MatrixXf tc(3,1); // translation matrix dron-camera
Eigen::MatrixXf rc(3,3); // rotation matrix dron-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix

Eigen::MatrixXf Spxd(8,1);   // 4 puntos x,y en pixeles que la deteccion de los paneles puntos acutales
Eigen::MatrixXf lambda(8,1);  // ganancias seleccionadas en el control

Eigen::MatrixXf q_vel(6,1) ; // velocidades del control servovisual
Eigen::MatrixXf q_vel_cam(6,1) ; // velocidades del control servovisual


///////////////////////////////////////callback
//void callback(const ImageConstPtr& in_RGB , const ImageConstPtr& in_depth, const OdometryConstPtr& odom_msg)
void callback(const ImageConstPtr& in_depth)
{
  auto t1 = Clock::now();

  cv_bridge::CvImagePtr cv_rgbCam , cv_depthCam;
      try
      {
        //cv_rgbCam   = cv_bridge::toCvCopy(in_RGB, sensor_msgs::image_encodings::BGR8);         // imagen de color de la camara 
        cv_depthCam = cv_bridge::toCvCopy(in_depth, sensor_msgs::image_encodings::TYPE_16UC1);  // imagen de profundida de la camara 
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

  //cv::Mat img_rgbCam    = cv_rgbCam->image;
  cv::Mat depth_img  = cv_depthCam->image;

  int cols_img = in_depth->width;
  int rows_img = in_depth->height;

  cv::Mat imgGrayscale;        // grayscale of input image
  //cv::Mat imgBlurred;            // intermediate blured image
  cv::Mat imgCanny;            // Canny edge image

 // cv::cvtColor(img_rgbCam, imgGrayscale, CV_BGR2GRAY);        // convert to grayscale

   cv::Mat image_grayscale = depth_img.clone();
   image_grayscale.convertTo(image_grayscale, CV_8U, 1 / 256.0);

  //cv::GaussianBlur(image_grayscale,     // input image
  //   imgBlurred,                        // output image
  //  cv::Size(5, 5),                     // smoothing window width and height in pixels
  // 1.5);                                // sigma value, determines how much the image will be blurred

  cv::Canny(image_grayscale,        // input image
      imgCanny,                    // output image
      minCan,                      // low threshold
      maxCan);                     // high threshold


  // Definir el kernel para la operación de dilatación
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

  // Aplicar la operación de dilatación a la imagen binarizada
  cv::Mat dilatedImage;
  cv::dilate(imgCanny, imgCanny, kernel);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(imgCanny, lines, rho, theta, threshold, minlenli, maxlenli);

  cv::Mat filtered_image = cv::Mat::zeros(rows_img, cols_img, CV_8UC1);
 
  for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        float angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180 / CV_PI;
        if (std::abs(angle) < 120 && std::abs(angle) > 60 ) {
            cv::line(filtered_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
        }
  }

  // Obtener las dimensiones de la imagen
  int imageHeight = filtered_image.rows;
  int imageWidth = filtered_image.cols;

  // Definir la región de interés en la mitad inferior de la imagen
  cv::Rect roi(0, 0, imageWidth, imageHeight / 2);
  // Rellenar la región de interés con ceros
  filtered_image(roi) = 0;

  cv::Mat kernel_dil = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 11));

  // Aplicar la operación de dilatación a la imagen binarizada
  cv::dilate(filtered_image, filtered_image, kernel_dil);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(filtered_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Vec4f> linesap;
  for (const auto& contour : contours) {
      // Ajustar una línea a los contornos
      cv::Vec4f line;
      cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);
      // Guardar la línea
      linesap.push_back(line); 
  }

  // Dibujar las líneas aproximadas en la imagen original
  cv::Mat resultImage;
  cv::cvtColor(filtered_image, resultImage, cv::COLOR_GRAY2BGR);
  float ang1 = 0 ,ang2 = 0, x11 = 0, x12 = 0, y11 =0 , y12 =0, x21 = 0, x22 = 0, y21 =0 , y22 =0 , vxl1 = 0, vxl2 = 0, vyl1 = 0, vyl2 = 0, xl1 =0, yl1 = 0, xl2=0, yl2 = 0;
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
        vxl1 = vx;
        vyl1 = vy;
        xl1 = x;
        yl1 =y;

        if (vy>0){
          ang1 = ang1-180;
          
        }
        ang_ori1 = ang1;
        std::cout<<"Line1: "<<line[0]<<", "<<line[1]<<", "<<line[2]<<", "<<line[3]<<std::endl;  
        flag_lin1 = true;    
      }
      else{
        
        cv::line(resultImage, pt1, pt2, cv::Scalar(0, 255, 0), 2);
        x21 = pt1.x;
        y21 = pt1.y; 
        x22 = pt2.x;
        y22 = pt2.y; 
        vxl2 = vx;
        vyl2 = vy;
        xl2 = x;
        yl2 = y;

        ang2 = std::atan2(y22-y21, x22-x21) * 180 / CV_PI;

        if (vy>0){
          ang2 = ang2-180;
        }
        ang_ori2 = ang2;

      }
      std::cout<<"Line2: "<<line[0]<<", "<<line[1]<<", "<<line[2]<<", "<<line[3]<<std::endl;
      flag_lin2 = true;    

  }
  std::cout<<"vyl2: "<<vyl1<<" vyl2:"<<vyl2<<" yl1: "<<yl1<<" yl2:"<<yl2<<std::endl;

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


  cv::Mat depthRoi;
  // Obtener la región de interés en la imagen de profundidad
  if (yc != 0 && xc != 0)
    depthRoi = depth_img(cv::Range(yc-100, yc+100), cv::Range(xc-2, xc+2));
  else
    depthRoi = depth_img(cv::Range(imageHeight/2 - 10 , imageHeight/2 + 10 ), cv::Range(imageWidth/2 - 10, imageWidth/2+10 ));

  // Calcular la profundidad promedio en el recuadro
  double depthSum = cv::sum(depthRoi)[0];
  double depthMean = (depthSum / (depthRoi.rows * depthRoi.cols))/1000.0;

  std::cout<<"Rows: "<<depthRoi.rows<< "Cols: "<<depthRoi.cols<<std::endl;


  // Obtener los valores de profundidad dentro del recuadro
  std::vector<double> depthValues;

  for (int ic=0; ic< depthRoi.rows; ic++){
      for (int jc=0; jc<depthRoi.cols ; jc++){
        auto px = depthRoi.at<ushort>(ic,jc);
            depthValues.push_back(px);
        }
    }

  // Calcular la mediana
  std::sort(depthValues.begin(), depthValues.end());
  int n = depthValues.size();

  std::cout<<"tamanno"<<depthValues.size()<<std::endl;

  double median;
  if (n % 2 == 0)
      median = (depthValues[n / 2 - 1] + depthValues[n / 2]) / 2.0;
  else
      median = depthValues[n / 2];


  double pixel_factor = 1000000.0/median;

  // puntos del cuadrado
  float p1_tx = (xc + pixel_factor*cos(ang_t));
  float p1_ty = (yc + pixel_factor*sin(ang_t));
  float p2_tx = (xc - pixel_factor*cos(ang_t));
  float p2_ty = (yc - pixel_factor*sin(ang_t));

  // puntos verticales
  float pv1_tx = (xc + pixel_factor*cos(ang_t-CV_PI/2));
  float pv1_ty = (yc + pixel_factor*sin(ang_t-CV_PI/2));
  float pv2_tx = (xc - pixel_factor*cos(ang_t-CV_PI/2));
  float pv2_ty = (yc - pixel_factor*sin(ang_t-CV_PI/2));

 
  float p3_tx = p1_tx - (pixel_factor*cos(ang_ori1-(2*ang_t)));
  float p3_ty = p1_ty + (pixel_factor*sin(ang_ori1+(2*ang_t)));
  float p4_tx = p2_tx - (pixel_factor*cos(ang_ori2-(2*ang_t)));
  float p4_ty = p2_ty + (pixel_factor*sin(ang_ori2+(2*ang_t)));


  
  // punto medio 
  cv::Point center(xc,yc);
  // puntos del cuadrado 
  cv::Point pt1_pl(p1_tx,p1_ty);
  cv::Point pt2_pl(p2_tx,p2_ty);
  cv::Point pt3_pl(p3_tx,p3_ty);
  cv::Point pt4_pl(p4_tx,p4_ty);

  std::cout<<"angt: "<<ang_t<<"ang1: "<<ang1<<"ang2: "<<ang2<<std::endl;
  std::cout<<"x1: " <<pt1_pl<<" p1: " <<std::endl;
  std::cout<<"x2: " <<pt2_pl<<" p2: " <<std::endl;
  std::cout<<"x3: " <<pt3_pl<<" p3: " <<std::endl;
  std::cout<<"x4: " <<pt4_pl<<" p4: " <<std::endl;

  // Dibujar puntos deseados // deberia sacar la inversa de los puntos deseados de spxd (cuando haya tiempo hacemos)
  cv::Point pt1_d(414, 242);
  cv::Point pt2_d(199, 237);
  cv::Point pt3_d(418, 135);
  cv::Point pt4_d(201, 129);

  cv::circle(resultImage, pt1_d, 5, cv::Scalar(0, 255, 0), -1);
  cv::circle(resultImage, pt2_d, 5, cv::Scalar(255, 0, 0), -1);
  cv::circle(resultImage, pt3_d, 5, cv::Scalar(0, 0, 255), -1);
  cv::circle(resultImage, pt4_d, 5, cv::Scalar(255, 0, 255),-1);

  cv::circle(resultImage, pt1_pl, 6, cv::Scalar(0, 255, 0), 1);
  cv::circle(resultImage, pt2_pl, 6, cv::Scalar(255, 0, 0), 1);
  cv::circle(resultImage, pt3_pl, 6, cv::Scalar(0, 0, 255), 1);
  cv::circle(resultImage, pt4_pl, 6, cv::Scalar(255, 0, 255),1);

  cv::circle(resultImage, center, 6, cv::Scalar(0, 0, 255),2);
  cv::circle(resultImage, cv::Point(int(imageWidth/2),int(imageHeight/2)), 2, cv::Scalar(255, 255, 255),2);

  cv::Point pv1(pv1_tx, pv1_ty);
  cv::Point pv2(pv2_tx, pv2_ty);

  cv::circle(resultImage, pv1, 3, cv::Scalar(0, 0, 255), 3);
  cv::circle(resultImage, pv2, 3, cv::Scalar(255, 0, 0),3);


  // calculo de angulo solo para verificar
  float angle_pixel = std::atan2(pt2_pl.x-pt1_pl.x,pt2_pl.y-pt1_pl.y);
  std::cout<<"angt: "<<ang_t<<" angle_pixel: "<<angle_pixel<<std::endl;


  /// Distancia perpendicular 

  


  // //===================================================== jacobiana imagen ================================================================

    float lx = Mc(0,0);// focal de la camarax
    float ly = Mc(1,1);// focal de la camaray
    float zr = depthMean;
    Eigen::MatrixXf Spx(8,1);   // 4 puntos x,y en pixeles que la deteccion de los paneles puntos acutales

    Spx(0,0) = pt1_pl.x;
    Spx(1,0) = pt1_pl.y;
    Spx(2,0) = pt2_pl.x;
    Spx(3,0) = pt2_pl.y;

    Spx(4,0) = pt3_pl.x;
    Spx(5,0) = pt3_pl.y;
    Spx(6,0) = pt4_pl.x;
    Spx(7,0) = pt4_pl.y;

     // cambio de sistema de referencias de los puntos del centro de imagen

  // matriz de traslaciones con el centro de la camara
  float cx = Mc(0,2);
  float cy = Mc(1,2);
  Eigen::MatrixXf Hc(4,4); 
  Hc <<  1,0,0,cx
        ,0,1,0,cy
        ,0,0,1,0
        ,0,0,0,1;

  Eigen::MatrixXf pin_Hc = Hc.completeOrthogonalDecomposition().pseudoInverse();

  Eigen::MatrixXf x_c (4,4);
  x_c<< Spx(0,0), Spx(2,0), Spx(4,0), Spx(6,0),
        Spx(1,0), Spx(3,0), Spx(5,0), Spx(7,0),
        0,        0,        0,        0,
        1,        1,        1,        1;

  // x_c<< Spx(0,0), Spx(2,0), 
  //       Spx(1,0), Spx(3,0), 
  //       0,        0,        
  //       1,        1       ;



  Eigen::MatrixXf x_ct (4,4);
  x_ct = pin_Hc * x_c;

  for (int i = 0;i<4;i++){
    for (int j = 0;j<4;j++){
      std::cout<< x_ct(i,j)<<", ";
    }
     std::cout<<std::endl;
  }


  Spx(0,0) = x_ct(0,0);
  Spx(1,0) = x_ct(1,0);
  Spx(2,0) = x_ct(0,1);
  Spx(3,0) = x_ct(1,1);
  Spx(4,0) = x_ct(0,2);
  Spx(5,0) = x_ct(1,2);
  Spx(6,0) = x_ct(0,3);
  Spx(7,0) = x_ct(1,3);  
  

    // Eigen::MatrixXf J_img(8,6); // jacobiana de imagen de 2 puntos de los objetos (2*2 x 6) matriz de medidas 4x6
    // J_img << -(lx/zr) ,0.0    ,Spx(0,0)/zr  , (Spx(0,0)*Spx(1,0))/lx     , -((lx+Spx(0,0)*Spx(0,0))/lx),Spx(1,0)
    //         ,0.0    ,(-ly/zr) ,Spx(1,0)/zr  , (ly+ (Spx(1,0)*Spx(1,0)))/ly  ,-(Spx(0,0)*Spx(1,0))/ly   ,-Spx(0,0)

    //         ,-(lx/zr) ,0.0    ,Spx(2,0)/zr  , (Spx(2,0)*Spx(3,0))/lx     ,-((lx+Spx(2,0)*Spx(2,0))/lx) , Spx(3,0)
    //         ,0.0    ,(-ly/zr) ,Spx(3,0)/zr  , (ly+ Spx(3,0)*Spx(3,0))/ly  ,-(Spx(2,0)*Spx(3,0))/ly     ,-Spx(2,0);

    //         ,-(lx/zr) ,0.0    ,Spx(4,0)/zr  , (Spx(4,0)*Spx(5,0))/lx     ,-((lx+Spx(4,0)*Spx(4,0))/lx) , Spx(5,0)
    //         ,0.0    ,(-ly/zr) ,Spx(5,0)/zr  , (ly+ Spx(5,0)*Spx(5,0))/ly  ,-(Spx(4,0)*Spx(5,0))/ly     ,-Spx(4,0)

    //         ,-(lx/zr) ,0.0    ,Spx(6,0)/zr  , (Spx(6,0)*Spx(7,0))/lx     ,-((lx+Spx(6,0)*Spx(6,0))/lx) , Spx(7,0)
    //         ,0.0    ,(-ly/zr) ,Spx(7,0)/zr  , (ly+ Spx(7,0)*Spx(7,0))/ly  ,-(Spx(6,0)*Spx(7,0))/ly     ,-Spx(6,0);

    Eigen::MatrixXf J_img(8,6); // jacobiana de imagen de 4 puntos de los objetos (4*2 x 6) matriz de medidas 8x6
    J_img << -(lx/zr) ,0.0    ,Spx(0,0)/zr  , (Spx(0,0)*Spx(1,0))/lx          ,-((lx*lx+Spx(0,0)*Spx(0,0))/lx)  , Spx(1,0)
            ,0.0    ,(-ly/zr) ,Spx(1,0)/zr  , (ly*ly+ (Spx(1,0)*Spx(1,0)))/ly ,-(Spx(0,0)*Spx(1,0))/ly          ,-Spx(0,0)

            ,-(lx/zr) ,0.0    ,Spx(2,0)/zr  , (Spx(2,0)*Spx(3,0))/lx          ,-((lx*lx+Spx(2,0)*Spx(2,0))/lx)  , Spx(3,0)
            ,0.0    ,(-ly/zr) ,Spx(3,0)/zr  , (ly*ly+ (Spx(3,0)*Spx(3,0)))/ly   ,-(Spx(2,0)*Spx(3,0))/ly        ,-Spx(2,0)

            ,-(lx/zr) ,0.0    ,Spx(4,0)/zr  , (Spx(4,0)*Spx(5,0))/lx          ,-((lx*lx+Spx(4,0)*Spx(4,0))/lx)  , Spx(5,0)
            ,0.0    ,(-ly/zr) ,Spx(5,0)/zr  , (ly*ly+ (Spx(5,0)*Spx(5,0)))/ly  ,-(Spx(4,0)*Spx(5,0))/ly         ,-Spx(4,0)

            ,-(lx/zr) ,0.0    ,Spx(6,0)/zr  , (Spx(6,0)*Spx(7,0))/lx          ,-((lx*lx+Spx(6,0)*Spx(6,0))/lx)  , Spx(7,0)
            ,0.0    ,(-ly/zr) ,Spx(7,0)/zr  , (ly*ly+ (Spx(7,0)*Spx(7,0)))/ly ,-(Spx(6,0)*Spx(7,0))/ly          ,-Spx(6,0);

  //==========================================================================================================================================
   
  
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
    
    Eigen::MatrixXf Jcam_robot(8,6);
    Jcam_robot = J_img * T_n;

    Eigen::MatrixXf pin_Jimg = J_img.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXf pin_Jc_r = Jcam_robot.completeOrthogonalDecomposition().pseudoInverse();

   //q_vel_cam = - pin_Jimg * (Spx - Spxd); // Velocidades de la camara 

    ///////////////////////////LEY DE CONTROL SERVO VISUAL

    Eigen::MatrixXf Sptan(8,1);
    Sptan(0,0) = lambda(0)* tanh((zr/((lx+ly)/2))*(Spx(0,0)-Spxd(0,0))); 
    Sptan(1,0) = lambda(1)* tanh((zr/((lx+ly)/2))*(Spx(1,0)-Spxd(1,0))); 
    Sptan(2,0) = lambda(2)* tanh((zr/((lx+ly)/2))*(Spx(2,0)-Spxd(2,0))); 
    Sptan(3,0) = lambda(3)* tanh((zr/((lx+ly)/2))*(Spx(3,0)-Spxd(3,0))); 
    Sptan(4,0) = lambda(4)* tanh((zr/((lx+ly)/2))*(Spx(4,0)-Spxd(4,0))); 
    Sptan(5,0) = lambda(5)* tanh((zr/((lx+ly)/2))*(Spx(5,0)-Spxd(5,0))); 
    Sptan(6,0) = lambda(6)* tanh((zr/((lx+ly)/2))*(Spx(6,0)-Spxd(6,0))); 
    Sptan(7,0) = lambda(7)* tanh((zr/((lx+ly)/2))*(Spx(7,0)-Spxd(7,0)));

    Eigen::MatrixXf Sperr(8,1);

    Sperr(0,0) = (Spx(0,0)-Spxd(0,0)); 
    Sperr(1,0) = (Spx(1,0)-Spxd(1,0)); 
    Sperr(2,0) = (Spx(2,0)-Spxd(2,0)); 
    Sperr(3,0) = (Spx(3,0)-Spxd(3,0)); 
    Sperr(4,0) = (Spx(4,0)-Spxd(4,0)); 
    Sperr(5,0) = (Spx(5,0)-Spxd(5,0)); 
    Sperr(6,0) = (Spx(6,0)-Spxd(6,0)); 
    Sperr(7,0) = (Spx(7,0)-Spxd(7,0)); 

    //////////////////// Nueva jacobiana

    // Eigen::MatrixXf J_theta(1,4);
    // Eigen::MatrixXf J_th_im(1,6);
    // float v1 = pt1_pl.y;
    // float v2 = pt2_pl.y;
    // float u1 = pt1_pl.x;
    // float u2 = pt2_pl.x;
    // J_theta << (v1-v2),-(u1-u2),-(v1-v2),(u1-u2);

    // J_theta = 1/(pow((v2-v1),2)+pow((u2-u1),2)) * J_theta;

    // Eigen::MatrixXf theta_p(1,1);
    // theta_p << lambda(4)*(angle_pixel - angle_desired);

    // J_th_im = J_theta * Jcam_robot;

    // Eigen::MatrixXf pin_J_th_im = J_th_im.completeOrthogonalDecomposition().pseudoInverse();

    std::cout<<"Norma error: "<<(zr/((lx+ly)/2))*Sperr.norm()<<std::endl;
    // std::cout<<"thetha_p: "<<theta_p<<std::endl;
    

    //lambda(0)+2.0/(1+(zr/((lx+ly)/2))*Sperr.norm())    0.0/(1+(zr/((lx+ly)/2))*Sperr.norm()),
    

   // q_vel =  - pin_Jc_r * (Spx - Spxd);  //calculo de velocidades de salida que van al robot 

    q_vel =  - pin_Jc_r * (Sptan);  //calculo de velocidades de salida que van al robot 

    // //// nueva jacobiana aumentada
    // Eigen::MatrixXf J_aumen(5,6);

    // J_aumen << Jcam_robot(0,0), Jcam_robot(0,1), Jcam_robot(0,2), Jcam_robot(0,3), Jcam_robot(0,4), Jcam_robot(0,5),
    //            Jcam_robot(1,0), Jcam_robot(1,1), Jcam_robot(1,2), Jcam_robot(1,3), Jcam_robot(1,4), Jcam_robot(1,5),
    //            Jcam_robot(2,0), Jcam_robot(2,1), Jcam_robot(2,2), Jcam_robot(2,3), Jcam_robot(2,4), Jcam_robot(2,5),
    //            Jcam_robot(3,0), Jcam_robot(3,1), Jcam_robot(3,2), Jcam_robot(3,3), Jcam_robot(3,4), Jcam_robot(3,5),
    //               J_th_im(0,0),    J_th_im(0,1),    J_th_im(0,2),    J_th_im(0,3),    J_th_im(0,4),    J_th_im(0,5);

    // Eigen::MatrixXf pin_J_aumen = J_aumen.completeOrthogonalDecomposition().pseudoInverse();

    // Eigen::MatrixXf x_p (5,1) ;

    // x_p << Sptan(0,0), Sptan(1,0), Sptan(2,0), Sptan(3,0), theta_p(0,0); 
    
    // q_vel = -pin_J_aumen * x_p;



   // q_vel =  - pin_J_th_im * theta_p;  //calculo de velocidades de salida que van al robot solo para el angulo theta

    //q_vel << q_vel(0,0), q_vel(1,0),q_vel(2,0),
           //  q_vel(3,0), q_vel(4,0) ,q_vel(5,0); // ganancias apra velocidades de salida

  sensor_msgs::ImagePtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resultImage).toImageMsg();  
  image_msg->header.stamp  = in_depth->header.stamp;
 
  featuresRGB_pub.publish(image_msg);

  if (xc == 0)
  q_vel << 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0 ;

  //////////// Envio de resultados de velocidades al topic de odometria 

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
  
  auto t2= Clock::now();
  std::cout<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()/1000000.0<<std::endl;

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

  nh.getParam("/rho", rho);
  nh.getParam("/theta", theta);
  nh.getParam("/threshold", threshold);
  nh.getParam("/minlen", minlenli);
  nh.getParam("/maxlen", maxlenli);

  nh.getParam("/control_ok", control_ok);
  nh.getParam("/angle_desired", angle_desired);

    
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
  //lambda <<  (double)param[0], (double)param[1], (double)param[2], (double)param[3], (double)param[4];
  
  //message_filters::Subscriber<Image>  rgbCam_sub (nh, imgTopic, 5);
  //message_filters::Subscriber<Image>  depthCam_sub(nh, depthTopic, 5);
  //message_filters::Subscriber<Odometry>     odom_robot(nh, odom_dron, 5);

  //typedef sync_policies::ApproximateTime<Image, Image, Odometry> MySyncPolicy;
  //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgbCam_sub, depthCam_sub, odom_robot);
  //sync.registerCallback(boost::bind(&callback, _1, _2, _3));


  ros::Subscriber sub = nh.subscribe<Image>(depthTopic, 10, callback);

  featuresRGB_pub = nh.advertise<sensor_msgs::Image>("/imgFeatures", 10);
  //featurespcl_pub= nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/pcl_features", 10);
  twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  
 
  ros::spin();
}