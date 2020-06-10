# Multi-ORB SLAM

修改自 ORB-SLAM2

# 1. Prerequisites

 系统 **Ubuntu 12.04**, **14.04** and **16.04** 等

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

## ROS (optional)

We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.

# 2. Building ORB-SLAM2 library and examples

Clone the repository:

```
git clone https://github.com/xxx/Multi_ORB_SLAM.git Multi_ORB_SLAM
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:

```
cd Multi_ORB_SLAM
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **rgbd_tum** in *Examples* folder.


# 3. RGB-D Example

1. 需要自己建立数据集.

对两个相机的数据集分别进行associate:

  ```
python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

```
python associate.py PATH_TO_SEQUENCE/rgb2.txt PATH_TO_SEQUENCE/depth2.txt > associations2.txt
```

2. Execute the following command. 

  执行命令
  例如:

 ```
./Examples/RGB-D/rgbd_tum xxx/Multi_ORB_SLAM/Vocabulary/ORBvoc.txt xxx/Multi_ORB_SLAM/OtherFiles/multi.yaml <数据集目录路径> <数据集目录路径>/associations.txt <数据集目录路径>/associations2.txt xxx/Multi_ORB_SLAM/OtherFiles/calibration.txt
 ```

# 4. ROS Examples

### Building the nodes for mono, monoAR, stereo and RGB-D

1. Add the path including *Examples/ROS/ORB_SLAM2* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2: 添加路径到环境变量

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/MUlti_ORB_SLAM/Examples/ROS
  ```

2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```

### Running RGB_D Node. 使用ROS运行RGBD节点进行实时 SLAM

安装奥比中光astra的ROS驱动,编写一个多相机launch文件,放在安装目录下,例如:

```
/opt/ros/kinetic/share/astra_launch/launch/下.
```


可以使用我编写的two_astra.launch,在/Multi_ORB_SLAM/OtherFiles/里.

使用launch文件运行多相机.

For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM2/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE PATH_TO_calibration_FILE
  ```

  也可以直接执行launch文件:
  ```
roslaunch orbslam2.launch
  ```

# 5. Processing your own sequences

You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the ORB-SLAM2 library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.
建立自己的多相机数据集

# 7. SLAM and Localization Modes

You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode

This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode

This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

````
ORB-SLAM2的原文档,see README_original.md 
````