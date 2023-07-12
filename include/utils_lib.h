#ifndef UTILS_LIB_H
#define UTILS_LIB_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdint>

uint16_t getPanelDistance(cv::Mat depthValues);
uint16_t getGroundDistance(cv::Mat depthValues);
uint16_t calculateClippingDistance(uint16_t groundDistance, uint16_t panelDistance);

cv::Mat findDrawContours(cv::Mat close,cv::Mat depthColormap);
std::vector<cv::Point> polyContour(std::vector<cv::Point> contour);
std::vector<cv::Point> simplifyContour(std::vector<cv::Point> contour);
std::vector<cv::Point> filterPolyLines(std::vector<cv::Point> contour);

cv::Mat filterDepth(cv::Mat depthValues, uint16_t clippingDistance);
std::pair<std::vector<cv::Point>, std::vector<cv::Point>> findLinesPatrick(cv::Mat close,cv::Mat depthMask);

std::pair<std::vector<cv::Point>, std::vector<cv::Point>> processDepth(cv::Mat depthValues);

#endif  // UTILS_LIB_H