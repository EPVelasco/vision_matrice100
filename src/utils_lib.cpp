#include "../include/utils_lib.h"

// Global filter Variables
const uint16_t poly_buffer_size = 15;
uint16_t poly_buffer_indx = 0;
uint16_t poly_buffer[4][2][poly_buffer_size] = {{{0}}};
std::vector<std::vector<cv::Point>>  contours;

uint16_t getPanelDistance(cv::Mat depthValues){
    std::vector<uint16_t> panelValues;
    for (int c = 292; c < 348; c++){
        for (int r = 15; r < 465; r++){
            uint16_t tempDepth = static_cast<uint16_t>(depthValues.at<uint16_t>(r, c));
            if(tempDepth != 0 /*&& tempDepth < 15000*/){
                panelValues.push_back(tempDepth);
            }
        }
    }
    // calculate median
    std::sort(panelValues.begin(), panelValues.end());
    int n = panelValues.size();
    int median = 0;
    if (n>0)
        if (n % 2 == 0)
            median = (panelValues[n / 2 - 1] + panelValues[n / 2]) / 2.0;
        else
            median = panelValues[n / 2];
    
    panelValues.clear();
    return median;
}

uint16_t getGroundDistance(cv::Mat depthValues){
    std::vector<uint16_t> groundValues;
    // aca en rows recorrer hasta 100 o 200
    for(int r = 15 ; r<200 ; r++){
        for(int c = 15; c < 70 ; c++){
            uint16_t tempDepth1 = depthValues.at<uint16_t>(r,c);
            uint16_t tempDepth2 = depthValues.at<uint16_t>(r,(640-c));
            if(tempDepth1 != 0 && tempDepth2 != 0){
                groundValues.push_back(static_cast<uint16_t>(tempDepth1));
                groundValues.push_back(static_cast<uint16_t>(tempDepth2));
            }
        }
    }

    // calculate median
    std::sort(groundValues.begin(), groundValues.end());
    uint16_t n = groundValues.size();
    uint16_t median = 0;
    if (n>0)
        if (n % 2 == 0)
            median = (groundValues[n / 2 - 1] + groundValues[n / 2]) / 2.0;
        else
            median = groundValues[n / 2];
    

    groundValues.clear();

    return median;
}

uint16_t calculateClippingDistance(uint16_t groundDistance/*, uint16_t panelDistance*/){
    // uint16_t  ratio = groundDistance - panelDistance;
    // if (ratio < 1000 || ratio > 3000 || panelDistance < 3000){
    //     return 5000;
    // }
    // uint16_t clippingDistance = (panelDistance+groundDistance)/2;
    uint16_t clippingDistance = groundDistance - 1000;
    return clippingDistance;
}

cv::Mat filterDepth(cv::Mat depthValues, uint16_t clippingDistance){
    for (uint16_t r = 0; r < 480; r++){
        for (uint16_t c = 0; c < 640; c++){
            uint16_t tempValue = static_cast<uint16_t>(depthValues.at<uint16_t>(r,c));
            if(tempValue > clippingDistance){
                // asignarle un cero en la misma posicion
                depthValues.at<uint16_t>(r,c) = 0;
            }
        }
    }

    cv::Mat grayScaleDepth;
    convertScaleAbs(depthValues, grayScaleDepth);
    
    // uint16_t thresh = 255;
    // uint16_t max_value = 255;
    // cv::Mat thresholded_image;
    // cv::threshold(grayScaleDepth, thresholded_image, thresh, max_value, cv::THRESH_TRUNC);

    // cv::Mat denoise;
    // cv::medianBlur(thresholded_image, denoise, 7);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::Mat close;
    //cv::morphologyEx(thresholded_image, close, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(grayScaleDepth, close, cv::MORPH_CLOSE, kernel);

    return close;
}

std::vector<cv::Point> simplifyContour(std::vector<cv::Point> contour){
    int n_corners=4;
    int n_iter = 0;
    int max_iter = 100;

    float lb = 0.0;
    float ub = 1.0;
    
    while(true){
        n_iter = n_iter+1;
        if(n_iter > max_iter){
            return contour; 
        }
        
        float k = (lb + ub)/2.0;
        float eps = k*cv::arcLength(contour, true);
        
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, eps, true);
        
        if (approx.size() > n_corners){
            lb = (lb + ub)/2.0;
        }else if (approx.size() < n_corners){
            ub = (lb + ub)/2.0;
        }else{
            return approx;
        }
    }
}

std::vector<cv::Point> filterPolyLines(std::vector<cv::Point> contour){
    //std::vector<cv::Point> filteredPoly;
    int filteredPoly[4][2];

    // anexar los puntos actuales a la posicion actual del array del filtro
    for(int i = 0 ; i<contour.size();i++){
        poly_buffer[i][0][poly_buffer_indx] = contour[i].x;
        poly_buffer[i][1][poly_buffer_indx] = contour[i].y;
    }
    poly_buffer_indx = (poly_buffer_indx+1)%poly_buffer_size;

    for(int r = 0 ; r<4;r++){
        for(int c = 0 ; c<2; c++){
            uint16_t temporal[poly_buffer_size] = {0};

            for(int d = 0 ; d<poly_buffer_size; d++){
                temporal[d] = poly_buffer[r][c][d];
            }
            
            // calculate median
            int endSize = sizeof(temporal) / sizeof(temporal[0]);
            std::sort(temporal, temporal + endSize);
            int n = sizeof(temporal) / sizeof(uint16_t);
            int median = 0;
            if (n % 2 == 0)
                median = (temporal[n / 2 - 1] + temporal[n / 2]) / 2.0;
            else
                median = temporal[n / 2];
            
            // prepare output 
            filteredPoly[r][c] = median;
        }
    }

    std::vector<cv::Point> gilada;
    for(int i = 0; i<4;i++){
        cv::Point temporalPoint(filteredPoly[i][0],filteredPoly[i][1]);
        gilada.push_back(temporalPoint);
    }
    return gilada;
}

std::vector<cv::Point> polyContour(std::vector<cv::Point> contour){
    std::vector<cv::Point> approximatedContour = simplifyContour(contour);
    std::vector<cv::Point> filteredPoly = filterPolyLines(approximatedContour);
    return filteredPoly;
}

cv::Mat findDrawContours(cv::Mat close,cv::Mat depthColormap){
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(close, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); // NONE vs SIMPLE
    
    for (size_t i = 0; i< contours.size(); i++){
        //drawContours( depthColormap, contours, (int)i, (0,255,0), 2, cv::LINE_8, hierarchy, 0 );
        double area = cv::contourArea(contours[i]);
        if (!(area < 150000 || area > 200000)){
            std::vector<cv::Point> approximatedContour = polyContour(contours[i]);
            cv::drawContours(depthColormap, std::vector<std::vector<cv::Point> >(1,approximatedContour), -1, CV_RGB(0,255,0), 2, 8);
        }
    }
    return depthColormap;   
}

std::pair<std::vector<cv::Point>, std::vector<cv::Point>> findLinesPatrick(cv::Mat close,cv::Mat depthMask){
    std::vector<cv::Point>  contour_left;
    std::vector<cv::Point> contour_right;

    for (int r = 15; r < 200; r++){
        bool flag_left = false;
        bool flag_rigth = false;
        for (int c = 15; c < 640/2; c++){
            int pixel_left  = static_cast<int>(close.at<uchar>(r, c));
            int pixel_right = static_cast<int>(close.at<uchar>(r, 640-c));
            
            if (pixel_left == 255 && flag_left == false){
                contour_left.push_back(cv::Point(c, r));
                flag_left = true;
            }
            
            if (pixel_right==255 && flag_rigth == false){
                contour_right.push_back(cv::Point(640-c, r));
                flag_rigth = true;
            }
            
            if (flag_rigth && flag_left)
                break;
        }
    }
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour_left);
    contours.push_back(contour_right);



    std::vector<cv::Vec4f> lines;
    for (const auto& contour : contours) {
        // Ajustar una línea a los contornos
        if (contour.size()==0)
            break;
        cv::Vec4f line;
        cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);
        lines.push_back(line); 
    }
    for (const auto& line : lines) {
      float vx = line[0];
      float vy = line[1];
      float x = line[2];
      float y = line[3];
      
      //Puntos de la linea
      cv::Point pt1_out_lin(x - 1000 * vx, y - 1000 * vy);
      cv::Point pt2_out_lin(x + 1000 * vx, y + 1000 * vy); 
      float angle = std::atan2(pt1_out_lin.y - pt2_out_lin.y, pt1_out_lin.x - pt2_out_lin.x) * 180 / CV_PI;

      if (std::abs(angle) < 135 && std::abs(angle) > 45 ) {
        //cv::line(depthMask, pt1_out_lin, pt2_out_lin, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        cv::line(depthMask, pt1_out_lin, pt2_out_lin, 255, 3, cv::LINE_AA);
      }
   }
   return {contour_left,contour_right};
}

std::pair<std::vector<cv::Point>, std::vector<cv::Point>> processDepth(cv::Mat depthValues){
    //uint16_t panelDistance = getPanelDistance(depthValues);
    uint16_t groundDistance = getGroundDistance(depthValues);
    uint16_t clippingDistance = calculateClippingDistance(groundDistance);    
    cv::Mat close = filterDepth(depthValues,clippingDistance);    

    cv::Mat depthMask = cv::Mat::zeros(close.size(), CV_8UC1);
    //depthMask = findDrawContours(close,depthMask);
    //contours.clear();
    // std::vector<cv::Point>  contour_left;
    // std::vector<cv::Point> contour_right;
   auto [contour_left, contour_right]  = findLinesPatrick(close,depthMask);
    

    return {contour_left,contour_right};
}