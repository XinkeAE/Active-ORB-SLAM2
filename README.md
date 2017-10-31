# Active-ORB-SLAM2
**Authors: ORB SLAM 2** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**Authors: Active ORB SLAM 2** Xinke Deng, Zixu Zhang, Avishai Sintov, Jing Huang, and Timothy Bretl

#1. Prerequisites
We have tested the library in **14.04** with ROS indigo. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## ROS 
ROS indigo is required [ros](http://wiki.ros.org/indigo/Installation/Ubuntu).

## OctoMap
OctoMap is required [octomap](https://github.com/OctoMap/octomap)

## OMPL is required
OMPL is required [OMPL](http://ompl.kavrakilab.org/download.html)

#2. Building Active-ORB-SLAM2 library

Clone the repository:
```
git clone https://github.com/XinkeAE/Actuve-ORB-SLAM2.git
```

```
cd Active-ORB-SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

#3. Building ROS
```
chmod +x build_ros.sh
./build_ros.sh
```

#4. Specify the goal pose in planning.cc and system.cc
changing line 265 - 268 in system.cc

#5. Run ros driver by running the script
```
./kinect.sh
```


