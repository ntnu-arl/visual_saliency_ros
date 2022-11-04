/*
* Developed by Tung Dang, University of Nevada, Reno.
* Ported from Robot Operating System library.
*/

#include <saliency_detection/saliencyDetectionVOCUS2.h>

void saliencyMapVOCUS2::imageCB(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	cv_bridge::CvImagePtr cv_ptr;
	sensor_msgs::Image salmap_;
	geometry_msgs::Point salientpoint_;

	Mat image_, saliencymap_, saliencymap_norm_;
	Point pt_salient;
	double maxVal;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	cv_ptr->image.copyTo(image_);

	// compute saliency map
	saliencymap_.create(image_.size(),CV_8UC1);

	if (calculateSaliencyMap(image_, saliencymap_, saliencymap_norm_, true))
	{

		//-- Return most salient POINT --//
		cv::minMaxLoc(saliencymap_,NULL,&maxVal,NULL,&pt_salient);
		salientpoint_.x = pt_salient.x;
		salientpoint_.y = pt_salient.y;

		//	CONVERT FROM CV::MAT TO ROSIMAGE FOR PUBLISHING
		saliencymap_.convertTo(saliencymap_, CV_8UC1,255);

		fillImage(salmap_, "mono8",saliencymap_.rows, saliencymap_.cols,
												saliencymap_.step, const_cast<uint8_t*>(saliencymap_.data));
	  salmap_.header.frame_id = msg_ptr->header.frame_id;
		salmap_.header.stamp = msg_ptr->header.stamp;
		saliencymap_pub_.publish(salmap_);

		fillImage(salmap_, "mono8",saliencymap_norm_.rows, saliencymap_norm_.cols, saliencymap_norm_.step, const_cast<uint8_t*>(saliencymap_norm_.data));
		salmap_.header.frame_id = msg_ptr->header.frame_id;
		salmap_.header.stamp = msg_ptr->header.stamp;
		saliencymap_normalize_pub_.publish(salmap_);

		point_pub_.publish(salientpoint_);
	}
	// if (saliencyMapVOCUS2::calculateSaliencyMap(image_, saliencymap_))
	// {
	// 	if (time_show) cout << "Computation time    : " << saliencyMapVOCUS2::getCalculateTime() << "\n";
	//
	// 	//-- Return most salient POINT --//
	// 	cv::minMaxLoc(saliencymap_,NULL,&maxVal,NULL,&pt_salient);
	// 	salientpoint_.x = pt_salient.x;
	// 	salientpoint_.y = pt_salient.y;
	//
	// 	//	CONVERT FROM CV::MAT TO ROSIMAGE FOR PUBLISHING
	// 	saliencymap_.convertTo(saliencymap_, CV_8UC1,255);
	// 	// equal
	// 	//cv::equalizeHist( saliencymap_, saliencymap_ );
	// 	// Normalize image
	// 	normalizeSaliencyMap(saliencymap_, saliencymap_norm_);
	//
	// 	fillImage(salmap_, "mono8",saliencymap_.rows, saliencymap_.cols,
	// 											saliencymap_.step, const_cast<uint8_t*>(saliencymap_.data));
	//   salmap_.header.frame_id = msg_ptr->header.frame_id;
	// 	salmap_.header.stamp = msg_ptr->header.stamp;
	// 	saliencymap_pub_.publish(salmap_);
	//
	// 	fillImage(salmap_, "mono8",saliencymap_norm_.rows, saliencymap_norm_.cols, saliencymap_norm_.step, const_cast<uint8_t*>(saliencymap_norm_.data));
	// 	salmap_.header.frame_id = msg_ptr->header.frame_id;
	// 	salmap_.header.stamp = msg_ptr->header.stamp;
	// 	saliencymap_normalize_pub_.publish(salmap_);
	//
	// 	point_pub_.publish(salientpoint_);
	//
	// 	//std::cout << "Sal computation time: " << calculateTime << std::endl;
	//
	// }
	else
	{
		ROS_WARN("Can not compute saliency map!!!");
	}
	return;
}

void saliencyMapVOCUS2::init(void)
{
	saliencyConfig.normalize = false;
	saliencyAlgorithm.setCfg(saliencyConfig);
}

void saliencyMapVOCUS2::normalizeSaliencyMap(const Mat& src, Mat& dst)
{
	cv::equalizeHist(src, dst);
	/* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */
  threshold( dst, dst, 256*(1 - thresholdPercent), 256, 3);
}

bool saliencyMapVOCUS2::calculateSaliencyMap(const Mat& src, Mat& dst)
{

	clock_t t1, t2;
	t1 = clock();

	// resize image
	Mat im_resize = src.clone();
	Size im_size_full(src.cols, src.rows);
	if (resizePercent < 1)
	{
		Size im_size(int((float)src.cols * resizePercent),
						int((float)src.rows * resizePercent));
		resize(src, im_resize, im_size, 0, 0);
	}

	saliencyAlgorithm.process(im_resize);

	bool CENTER_BIAS = false;
	if(CENTER_BIAS)
		dst = saliencyAlgorithm.add_center_bias(0.00005);
	else
		dst = saliencyAlgorithm.get_salmap();

	double minVal, maxVal;
	// threshold here
	threshold( dst, dst, thresholdSaliency, maxVal, cv::THRESH_TOZERO);

	// scale the image to range [0,1]
	minMaxLoc(dst, &minVal, &maxVal, NULL, NULL);
	if (maxVal != minVal)
		dst.convertTo(dst, CV_32FC1, 1.0/(maxVal-minVal), -minVal/(maxVal-minVal)); // scale the image

	// re-resize saliency image to full size
	if (resizePercent < 1){
		resize(dst, dst, im_size_full, 0, 0);
	}

	t2 = clock();
	float diff = ((float)t2-(float)t1);
	calculateTime = diff / CLOCKS_PER_SEC;

	return true;
}

bool saliencyMapVOCUS2::calculateSaliencyMap(const Mat& src, Mat& dst, Mat& dst_norm, bool norm_en)
{

	clock_t t1, t2;
	t1 = clock();

	// resize image
	Mat im_resize = src.clone();
	Size im_size_full(src.cols, src.rows);
	if (resizePercent < 1)
	{
		Size im_size(int((float)src.cols * resizePercent),
						int((float)src.rows * resizePercent));
		resize(src, im_resize, im_size, 0, 0);
	}

	saliencyAlgorithm.process(im_resize);

	bool CENTER_BIAS = false;
	if(CENTER_BIAS)
		dst = saliencyAlgorithm.add_center_bias(0.00005);
	else
		dst = saliencyAlgorithm.get_salmap();

	double minVal, maxVal;
	// threshold here
	threshold( dst, dst, thresholdSaliency, maxVal, cv::THRESH_TOZERO);

	// scale the image to range [0,1]
	minMaxLoc(dst, &minVal, &maxVal, NULL, NULL);
	if (maxVal != minVal)
		dst.convertTo(dst, CV_32FC1, 1.0/(maxVal-minVal), -minVal/(maxVal-minVal)); // scale the image

	// re-resize saliency image to full size
	if (resizePercent < 1){
		resize(dst, dst, im_size_full, 0, 0);
	}

	// scale the image to range [0, 255] for visualization
	Mat dst_to_norm = dst.clone();
	dst_to_norm.convertTo(dst_norm, CV_8UC1,255);

	t2 = clock();
	float diff = ((float)t2-(float)t1);
	calculateTime = diff / CLOCKS_PER_SEC;

	if (time_show) cout << "Computation time    : " << calculateTime << "\n";

	return true;
}

bool saliencyMapVOCUS2::calculateSaliencyMap(const Mat& src, Mat& dst, Mat& vis)
{

	// clock_t t1, t2;
	// t1 = clock();
	Mat img_norm;
	saliencyMapVOCUS2::calculateSaliencyMap(src, dst, img_norm, true);

	// Make a blend image for visualization
	Mat color_image = dst.clone();
	cvtColor(color_image, color_image, CV_GRAY2BGR );
	color_image.convertTo(vis, CV_32FC3);

	Mat src_;
	src.convertTo(src_, CV_32FC3);
	vis = vis.mul(src_);

	// double min, max;
	// minMaxLoc(vis, &min, &max, NULL, NULL);
	// vis.convertTo(vis, CV_32FC3, 1/double(max),0);

	// t2 = clock();
	// float diff = ((float)t2-(float)t1);
	// calculateTime = diff / CLOCKS_PER_SEC;

	return true;
}

void saliencyMapVOCUS2::runTest(int test_option)
{
	if (test_option == 0) return;
	if (testFolder == "default_value") return;

	if (test_option == 1)
	{
		if (testImage == "default_value") return;

		cout << "Working folder      : " << testFolder << "\n";
		cout << "Load image          : " << testImage << "\n";
		Mat image_rgb = cv::imread(testFolder+testImage);
		Mat image_sal, image_blend;

		saliencyMapVOCUS2::calculateSaliencyMap(image_rgb, image_sal, image_blend);

		//	convert to full image for saving
		cout << "Save saliency map   : map_VOCUS2_" << testImage << "\n";
		image_sal.convertTo(image_sal, CV_8UC1,255);

		stringstream image_write_name;
		image_write_name << testFolder << "map_raw_VOCUS2_" << testImage;
		cv::imwrite(image_write_name.str(), image_sal);

		cv::equalizeHist( image_sal, image_sal );
		stringstream image_write_name2;
		image_write_name2 << testFolder << "map_equalize_VOCUS2_" << testImage;
		cv::imwrite(image_write_name2.str(), image_sal);

		//image_blend.convertTo(image_blend, CV_8UC3, 255);
		cout << "Save overlay image  : vis_VOCUS2_" << testImage << "\n";
		stringstream image_write_name1;
		image_write_name1 << testFolder << "vis_VOCUS2_" << testImage;
		cv::imwrite(image_write_name1.str(), image_blend);

		//
		cout << "Computation time    : " << saliencyMapVOCUS2::getCalculateTime() << "\n";
		cout << "---------------------\n";

	}
	else if (test_option == 2)
	{
		int count = 1;
		bool cont = true;
		cout << "Working folder      : " << testFolder << "\n";
		while(cont)
		{
			stringstream image_name;
			image_name << testFolder << count << testImageType;

			cout << "Load image          : " << count << testImageType << "\n";
			Mat image_rgb = cv::imread(image_name.str());
			if (image_rgb.empty())
			{
				cout << "STOPPED             : can not load image! \n";
				cont = false;
				break;
			}

			// resize to 640x480
			cout << "Resize image to     : 640x480" << "\n";
			Size im_standard_size(640, 480);
	    resize(image_rgb, image_rgb, im_standard_size, 0, 0);

			Mat image_sal, image_blend;
			saliencyMapVOCUS2::calculateSaliencyMap(image_rgb, image_sal, image_blend);

			//	convert to full image for saving
			cout << "Save saliency map   : map_VOCUS2_" << count << testImageType << "\n";
			image_sal.convertTo(image_sal, CV_8UC1,255);
			cv::equalizeHist( image_sal, image_sal );
			stringstream image_write_name;
			image_write_name << testFolder << "map_VOCUS2_" << count << testImageType;
			cv::imwrite(image_write_name.str(), image_sal);

			//image_blend.convertTo(image_blend, CV_8UC3, 255);
			cout << "Save overlay image  : vis_VOCUS2_" << count << testImageType << "\n";
			stringstream image_write_name1;
			image_write_name1 << testFolder << "vis_VOCUS2_" << count << testImageType;
			cv::imwrite(image_write_name1.str(), image_blend);

			//
			cout << "Computation time    : " << saliencyMapVOCUS2::getCalculateTime() << "\n";
			cout << "---------------------\n";
			count++;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "saliencymap");

	saliencyMapVOCUS2 salmapVOCUS2;
	salmapVOCUS2.init();

	ros::spin();

	return 0;
}
