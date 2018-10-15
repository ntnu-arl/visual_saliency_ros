#ifndef SALIENCY_NODE_H_
#define SALIENCY_NODE_H_

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

#include "saliency_detection.h"

class SaliencyNode {
protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher sal_map_pub_;
  image_transport::Publisher sal_map_norm_pub_;

private:
  SaliencyConfig SalConf;
  SaliencyDetection SalDet;
  void ImageCb(const sensor_msgs::ImageConstPtr &msg_ptr);

public:
  SaliencyNode() : nh_("~"), it_(nh_) {
    image_sub_ = it_.subscribe("/saliency_detection/rgbimage_in", 1,
                               &SaliencyNode::ImageCb, this);
    sal_map_pub_ = it_.advertise("/saliency_detection/image", 1);
    sal_map_norm_pub_ = it_.advertise("/saliency_detection/image_norm", 1);

    std::string alg;
    nh_.param<std::string>("saliency_algorithm", alg, "");
    std::cout << "Algorithm: " << alg << std::endl;
    SalConf.saliency_algorithm = alg;

    double resize;
    nh_.param<double>("image_resize_percent", resize, 1);
    std::cout << "Image resize:" << resize << std::endl;
    SalConf.resize_percent = (resize < 1) ? resize : 1;

    double thres;
    nh_.param<double>("threshold_percent", thres, 1);
    std::cout << "Threshold precentage: " << thres << std::endl;
    SalConf.threshold_percent = ((thres <= 1) && (thres >= 0)) ? thres : 1;

    int tc_pixels = 0;
    nh_.param<int>("topcut_pixels", tc_pixels, 0);
    std::cout << "Cut " << tc_pixels << " pixels from the top" << std::endl;
    SalConf.topcut_pixels = tc_pixels;

    SalDet.Init(&SalConf);
  }

  ~SaliencyNode() { nh_.shutdown(); }
};

#endif // SALIENCY_NODE_H_
