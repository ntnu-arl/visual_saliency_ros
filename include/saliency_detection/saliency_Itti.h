//===================================================================================
// Name        : saliencyDetectionItti.h
// Author      : Oytun Akman, oytunakman@gmail.com
// Version     : 1.0
// Copyright   : Copyright (c) 2010 LGPL
// Description : C++ implementation of "A Model of Saliency-Based Visual
// Attention
//				 for Rapid Scene Analysis" by Laurent Itti, Christof
//Koch and Ernst
//				 Niebur (PAMI 1998).
//===================================================================================

#ifndef SALIENCY_ITTI_H_
#define SALIENCY_ITTI_H_
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

class SaliencyItti {

public:
  void calculateSaliencyMap(const Mat &src, Mat &dst, int scaleBase);
  void combineFeatureMaps(int scale);

  Mat conspicuityMap_I;
  Mat conspicuityMap_C;
  Mat conspicuityMap_O;
  Mat S;

private:
  Mat r, g, b, R, G, B, Y, I;
  vector<Mat> gaussianPyramid_I;
  vector<Mat> gaussianPyramid_R;
  vector<Mat> gaussianPyramid_G;
  vector<Mat> gaussianPyramid_B;
  vector<Mat> gaussianPyramid_Y;

  void createChannels(const Mat *src);
  void createScaleSpace(const Mat *src, vector<Mat> *dst, int scale);

  void normalize_rgb();
  void create_RGBY();
  void createIntensityFeatureMaps();
  void createColorFeatureMaps();
  void createOrientationFeatureMaps(int orientation);
  void mapNormalization(Mat *src);
  void clearBuffers();

  vector<Mat> featureMaps_I;
  vector<Mat> featureMaps_RG;
  vector<Mat> featureMaps_BY;
  vector<Mat> featureMaps_0;
  vector<Mat> featureMaps_45;
  vector<Mat> featureMaps_90;
  vector<Mat> featureMaps_135;

};
#endif //SALIENCY_ITTI_H_
