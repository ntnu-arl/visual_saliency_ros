//===================================================================================
// Ported from Robot Operating System library

#pragma once

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <geometry_msgs/Point.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

// AIM
#include <VOCUS2.h>

using namespace cv;
using namespace std;

class saliencyMapVOCUS2
{
protected:
    	ros::NodeHandle nh_;
    	ros::Publisher point_pub_;
    	image_transport::ImageTransport it_;
    	image_transport::Subscriber 	image_sub_;
    	image_transport::Publisher		saliencymap_pub_;
      image_transport::Publisher		saliencymap_normalize_pub_;

public:
    	saliencyMapVOCUS2() : nh_("~"), it_(nh_)
    	{
    		image_sub_ = it_.subscribe("/rgbimage_in", 1, &saliencyMapVOCUS2::imageCB, this);
    		saliencymap_pub_= it_.advertise("/saliency/image", 1);
        saliencymap_normalize_pub_= it_.advertise("/saliency/image_norm", 1);
        point_pub_ = nh_.advertise<geometry_msgs::Point>("/saliency/salientpoint", 1);

        double resize;
        nh_.param<double>("image_resize_percent", resize, 1);
        std::cout << "Image resize:" << resize << '\n';
        resizePercent = (resize < 1)?resize:1;

        double thres;
        nh_.param<double>("threshold_percent", thres, 1);
        std::cout << "Threshold precentage: " << thres << '\n';
        thresholdPercent = ((thres <= 1) && (thres >= 0)) ? thres : 1;

        topcut_enable = false;
        nh_.param<bool>("topcut_enable", topcut_enable, false);
        std::cout << "Cut image: " << topcut_enable << '\n';

        topcut_pixels = 0;
        nh_.param<int>("topcut_pixels", topcut_pixels, 0);
        std::cout << "Cut " << topcut_pixels << " pixels from the top" << '\n';

        time_show = false;
        nh_.param<bool>("time_show", time_show, false);
        std::cout << "Computation time shows: " << time_show  << '\n';


        int test_option;
        nh_.param<int>("test_option", test_option, 0);
        if (test_option)
        {
          string test_folder;
          nh_.param<string>("test_folder", test_folder, "default_value");
          std::cout << "Folder for testing  :" << test_folder << '\n';
          testFolder = test_folder;

          nh_.param<string>("test_image_type", testImageType, ".png");
          std::cout << "Image type          :" << testImageType << '\n';


          string test_image;
          nh_.param<string>("test_image", test_image, "default_value");
          std::cout << "Image for testing   :" << test_image << '\n';
          testImage = test_image;
        }
        testOption = test_option;

        init();

        runTest(testOption);
    	}

    	~saliencyMapVOCUS2()
    	{
    		nh_.shutdown();
    	}

      float getCalculateTime(void)
      {
        return calculateTime;
      }

      bool calculateSaliencyMap(const Mat& src, Mat& dst);
      bool calculateSaliencyMap(const Mat& src, Mat& dst, Mat& vis);
      void normalizeSaliencyMap(const Mat& src, Mat& dst);
      bool calculateSaliencyMap(const Mat& src, Mat& dst, Mat& dst_norm, bool norm_en);

private:
      double     resizePercent;
      VOCUS2_Cfg saliencyConfig;
      VOCUS2     saliencyAlgorithm;
      float      calculateTime;
      double thresholdPercent;

      void imageCB(const sensor_msgs::ImageConstPtr& msg_ptr);
      void init(void);

      int testOption; //0: no; 1: image; 2: folder
      string testFolder;
      string testImage;
      string testImageType;
      void runTest(int test_option);

      bool topcut_enable;
      int topcut_pixels;
      bool time_show;

};
