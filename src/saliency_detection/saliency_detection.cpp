/*
* Developed by Tung Dang, University of Nevada, Reno.
* Ported from Robot Operating System library.
*/
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>

#include "VOCUS2.h"
#include "saliency_detection.h"
#include "saliency_detection/saliency_Hou.h"
#include "saliency_detection/saliency_Itti.h"

void SaliencyDetection::Init(SaliencyConfig *conf) {
  SalConf.saliency_algorithm = conf->saliency_algorithm;
  SalConf.resize_percent = conf->resize_percent;
  SalConf.threshold_percent = conf->threshold_percent;
  SalConf.topcut_pixels = conf->topcut_pixels;
}

bool SaliencyDetection::Compute(const cv::Mat &src, cv::Mat &dst,
                                cv::Mat &dst_norm, bool norm_en) {
  clock_t t1, t2;
  t1 = clock();

  // Create the ROI
  cv::Rect roi(0, SalConf.topcut_pixels, src.cols,
               src.rows - SalConf.topcut_pixels);
  cv::Mat src_roi = src(roi);

  // Resize image
  cv::Mat im_resize = src_roi.clone();
  cv::Size im_size_full(src_roi.cols, src_roi.rows);
  if (SalConf.resize_percent < 1) {
    cv::Size im_size(int((float)src_roi.cols * SalConf.resize_percent),
                     int((float)src_roi.rows * SalConf.resize_percent));
    cv::resize(src_roi, im_resize, im_size, 0, 0);
  }

  if (SalConf.saliency_algorithm.compare("VOCUS2") == 0) {
    RunVOCUS2(im_resize, dst);
  } else if (SalConf.saliency_algorithm.compare("Itti") == 0) {
    RunItti(im_resize, dst);
  } else if (SalConf.saliency_algorithm.compare("Hou") == 0) {
    RunHou(im_resize, dst);
  } else {
    std::cout << SalConf.saliency_algorithm << std::endl;
    return false;
  }

  // Re-resize saliency image to full size
  if (SalConf.resize_percent < 1) {
    resize(dst, dst, im_size_full, 0, 0);
  }

  if (norm_en) {
    cv::Mat dst_to_norm = dst.clone();
    dst_to_norm.convertTo(dst_to_norm, CV_8UC1, 255);
    NormalizeSaliencyMap(dst_to_norm, dst_norm);
    cv::copyMakeBorder(dst_norm, dst_norm, SalConf.topcut_pixels, 0, 0, 0,
                       BORDER_CONSTANT, 0);
  }

  cv::copyMakeBorder(dst, dst, SalConf.topcut_pixels, 0, 0, 0, BORDER_CONSTANT,
                     0);

  t2 = clock();
  float diff = ((float)t2 - (float)t1);
  double calculateTime;
  calculateTime = diff / CLOCKS_PER_SEC;

  if (true)
    cout << "Computation time    : " << calculateTime << "\n";

  return true;
}

void SaliencyDetection::NormalizeSaliencyMap(const cv::Mat &src, cv::Mat &dst) {
  cv::equalizeHist(src, dst);
  /* 0: Binary
  1: Binary Inverted
  2: Threshold Truncated
  3: Threshold to Zero
  4: Threshold to Zero Inverted
  */
  cv::threshold(dst, dst, 256 * (1 - SalConf.threshold_percent), 256, 3);
}

void SaliencyDetection::RunVOCUS2(cv::Mat &im, cv::Mat &dst) {
  VOCUS2_Cfg saliencyConfig;
  VOCUS2 saliencyAlgorithm;

  saliencyAlgorithm.process(im);

  bool CENTER_BIAS = false;
  if (CENTER_BIAS)
    dst = saliencyAlgorithm.add_center_bias(0.00005);
  else
    dst = saliencyAlgorithm.get_salmap();

  double minVal, maxVal;
  minMaxLoc(dst, &minVal, &maxVal, NULL, NULL);
  if (maxVal != minVal)
    dst.convertTo(dst, CV_32FC1, 1.0 / (maxVal - minVal),
                  -minVal / (maxVal - minVal)); // scale the image
}

void SaliencyDetection::RunItti(cv::Mat &im, cv::Mat &dst) {
  SaliencyItti s;
  s.calculateSaliencyMap(im, dst, 1);
}

void SaliencyDetection::RunHou(cv::Mat &im, cv::Mat &dst) {
  SaliencyHou s;
  s.calculateSaliencyMap(&im, &dst);
}
