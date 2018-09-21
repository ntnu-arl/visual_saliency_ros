//===================================================================================
// Name        : saliencyDetectionHou.h
// Author      : Oytun Akman, oytunakman@gmail.com
// Version     : 1.0
// Copyright   : Copyright (c) 2010 LGPL
// Description : C++ implementation of "Saliency Detection: A Spectral Residual
//				 Approach" by Xiaodi Hou and Liqing Zhang (CVPR
//2007).
//===================================================================================

#ifndef SALIENCY_HOU_H_
#define SALIENCY_HOU_H_

// ROS
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class SaliencyHou {
public:
  void calculateSaliencyMap(const Mat *src, Mat *dst);
};
#endif //SALIENCY_HOU_H_
