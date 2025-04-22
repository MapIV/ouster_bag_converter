# ouster_bag_converter

The purpose of this package is to remove NaN points of Ouster PointCloud2 from the bag file and save it as a new bag file.

## Requirements

- Ubuntu 20.04
- [ROS Noetic](http://wiki.ros.org/noetic)
- [PCL](http://www.pointclouds.org/)

## Installation

```
mkdir -p ouster_bag_converter_ws/src
cd ouster_bag_converter_ws/src
git clone https://github.com/MapIV/ouster_bag_converter.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Usage

```
./deve/lib/ouster_bag_converter/ouster_bag_converter input_rosbag topic_name output_rosbag
```
