#include "saliency_node.h"

void SaliencyNode::ImageCb(const sensor_msgs::ImageConstPtr &msg_ptr) {
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::Image salmap_;
  sensor_msgs::Image salmap_norm_;
  cv::Mat image_proc;

  try {
    cv_ptr =
        cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  cv_ptr->image.copyTo(image_proc);

  cv::Mat sal_map(image_proc.rows, image_proc.cols, CV_8UC1,
                  cv::Scalar::all(0));
  cv::Mat sal_map_norm(image_proc.rows, image_proc.cols, CV_8UC1,
                      cv::Scalar::all(0));

  if (SalDet.Compute(image_proc, sal_map, sal_map_norm, true)) {
    sal_map.convertTo(sal_map, CV_8UC1, 255);
    fillImage(salmap_, "mono8", sal_map.rows, sal_map.cols, sal_map.step,
              sal_map.data);
    salmap_.header.frame_id = msg_ptr->header.frame_id;
    salmap_.header.stamp = msg_ptr->header.stamp;
    sal_map_pub_.publish(salmap_);

    fillImage(salmap_norm_, "mono8", sal_map_norm.rows, sal_map_norm.cols,
              sal_map_norm.step, sal_map_norm.data);
    salmap_norm_.header.frame_id = msg_ptr->header.frame_id;
    salmap_norm_.header.stamp = msg_ptr->header.stamp;
    sal_map_norm_pub_.publish(salmap_norm_);
  } else {
    ROS_ERROR("Can not compute saliency map!");
  }
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "saliency_detection");

  SaliencyNode sal_node;
  ros::spin();
  return 0;
}
