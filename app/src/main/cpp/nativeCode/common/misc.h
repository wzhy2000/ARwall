
#ifndef MISC_H
#define MISC_H

#include <stdio.h>
#include <string>
#include "myLogger.h"
#include "myGLM.h"
#include <opencv2/opencv.hpp>

std::string GetFileName(std::string fileName);

std::string GetDirectoryName(std::string fullFileName);

void PrintGLMMat4(glm::mat4 testMat);

void PrintCVMat(cv::Mat matToPrint);

std::vector<cv::Point2f> Keypoint2Point(std::vector<cv::KeyPoint> keypoints);

void CalcShiftedCorners(cv::Mat homography, int nWidth, int nHeight, float* vertices);

#endif //MISC_H
