/*
* Developed by Tung Dang, University of Nevada, Reno.
* Modified from Robot Operating System library.
*/
#ifndef SALIENCY_DETECTION_H_
#define SALIENCY_DETECTION_H_

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

struct SaliencyConfig {
  std::string saliency_algorithm;
  double resize_percent;
  double threshold_percent;
  int topcut_pixels;
};

class SaliencyDetection {
public:
  SaliencyDetection() {}
  ~SaliencyDetection() {}
  void Init(SaliencyConfig *conf);
  bool Compute(const cv::Mat &src, cv::Mat &dst, cv::Mat &dst_norm,
               bool norm_en);

private:
  SaliencyConfig SalConf;
  void RunVOCUS2(cv::Mat &im, cv::Mat &sal_map);
  void RunItti(cv::Mat &im, cv::Mat &sal_map);
  void RunHou(cv::Mat &im, cv::Mat &sal_map);
  void NormalizeSaliencyMap(const cv::Mat &src, cv::Mat &dst);
};
#endif // SALIENCY_DETECTION_H_
