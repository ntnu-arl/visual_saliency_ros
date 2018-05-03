# ROS Implementation for Visual Saliency Algorithms

## Build this repo in catkin workspace
```
git clone https://github.com/unr-arl/visual_saliency_ros
cd visual_saliency_ros
git submodule update --init --recursive
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
## Run an example
- Download an example rosbag from here: [link](https://drive.google.com/open?id=1LWJpi7UWKhBM7VNIopiblMV3YpGo_5AH)
- Run the launch file: `roslaunch visual_saliency saliency_vocus2.launch`

## References: 
You can find mode details in the paper: *"Traditional Saliency Reloaded: A Good Old Model in New Shape"*, S. Frintrop, T. Werner, G. Martin Garcia, in Proceedings of the IEEE International Conference on Computer Vision and Pattern Recognition (CVPR), 2015.

And also please check out the [VOCUS2 repo](https://github.com/GeeeG/VOCUS2/tree/9cd659874afbd6b228bcea92362c2dec62a34d73) for more information.
