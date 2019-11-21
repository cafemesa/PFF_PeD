# PFF_PeD
PointCloud Fast Filter for People Detection

The algorithm "PointCloud Fast Filter for People Detection" (PFF-PeD) detects people by processing effectively point-cloud data, independently of the specific 3D sensor employed. By focusing on the most informative observations, PFF-PeD is able to improve the accuracy and processing times compared to existing 2D and 3D solutions. We implemented PFF-PeD in C++ and integrated its operation in ROS Kinetic Kame in a laptop with Ubuntu 16.04.

## Subscribed Topics

### RGB-D Camera

* [/camera/depth/points] : Sensor PointCloud for peolple detection

### Velodyne

* [/velodyne_points] :  Sensor PointCloud for peolple detection



## Published Topics

* [/PeopleCloud] :  PointCloud with the final detection.
* [/TrunkCloud] :  PointCloud filtered of the trunk range.
* [/LegsCloud] :  PointCloud filtered of the Legs range.



## Parameters

* [resolution] :  Angle resolution in degrees for filter the points in a cloud of size 360°/resolution.
* [sensorheight] :  S<sub>h</sub>, see the following image. in meters.
* [legs_begin] :  F<sub>r</sub>, see the following image. in meters.
* [legs_end] :  F<sub>r</sub> + L<sub>r</sub>, see the following image. in meters.
* [trunk_begin] :  F<sub>r</sub> + L<sub>r</sub> + S<sub>r</sub>, see the following image. in meters.
* [trunk_end] :  F<sub>r</sub> + L<sub>r</sub> + S<sub>r</sub> + T<sub>r</sub>, see the following image. in meters.


![alt text](https://drive.google.com/uc?export=view&id=1TZIoPp-C2Put52MMIJFfRcFap0CeOHIg)

### Special parameters for RGB-D Camera

* [camera_fov] :  Horizontal Field of View of the RGB-D camera in degrees.

### Special parameters for Velodyne

* [min_vision_range] :  Minimum Detection Range, in meters.
* [max_vision_range] :  Maximum Detection Range, in meters.
* [horizontal_fov] :  Desired horizontal field of view (from -theta/2 to theta/2), see the following image. in degrees.

![alt text](https://drive.google.com/uc?export=view&id=1489zOF8vgnzcyRe783N9ieJLoMB6DqZj)


## How to build with catkin
```sh
$ cd ~/catkin_ws/src/
$ git clone https://github.com/cafemesa/PFF_PeD.git
$ cd ~/catkin_ws && catkin_make
```

## How to run

### Launch Files (roslaunch)

#### Requirements

To run the detection with the Astra Camera (from: https://github.com/orbbec/ros_astra_camera):

```sh
$ sudo apt install ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros
$ cd ~/catkin_ws/src
$ git clone https://github.com/orbbec/ros_astra_camera
$ ./ros_astra_camera/scripts/create_udev_rules
$ https://github.com/orbbec/ros_astra_launch.git
$ cd ~/catkin_ws && catkin_make
```

To run the detection with the Velodyne:

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/velodyne.git
$ cd ~/catkin_ws
$ rosdep install --from-paths src -i -y
$ catkin_make
```

#### Detection with Astra Camera PointCloud

#### Detection with Velodyne VLP-16 PointCloud

#### Detection with Velodyne VLP-16 PointCloud Bag

### Nodes (rosrun)



