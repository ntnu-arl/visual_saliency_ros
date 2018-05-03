# ROS Implementation for Visual Saliency Algorithms

## How to build
```
git clone https://github.com/unr-arl/visual_saliency_ros
cd visual_saliency_ros
git submodule update --init --recursive
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
## How to run
- Download an example rosbag from here: [link](https://drive.google.com/open?id=1LWJpi7UWKhBM7VNIopiblMV3YpGo_5AH)
- Run the launch file: `roslaunch visual_saliency saliency_vocus2.launch`
