# 
Second Robotics Project - Politecnico di Milano 2018/2019  

## Overview

## Project Structure
The project contains an "src" folder with our odometry node, a "launch" folder containing the launch file to run all the needed nodes and a "param" folder containing the configuration files for each one of the nodes.

```
filters_project
    ├── CMakeLists.txt
    ├── launch
    │   ├── filters_project.launch
    │   └── filters_project_with_bag.launch
    ├── package.xml
    ├── params
    │   ├── ekf_params.yaml
    │   ├── imu_filter_params.yaml
    │   └── navsat_transform_params.yaml
    └── src
        └── node.cpp
```

## Nodes and Topics
We produced a single node that subscribes to the required topic ("/speedsteer") to retrieve the car's velocity and steering angle, in order to compute its odometry. The odometry is then published on the "/car_odometry" topic. 
The node also published a tf transform from the "odom" frame to the "base_link" one.  
  
The project also instantiates:
  * a COMPLEMENTARY FILTER node to filter IMU data read from "/swiftnav/rear/imu" topic. We used the version downloaded from official repositories. We needed to remap the "/imu" topic required by the filter to "/swiftnav/rear/imu". The filter then publishes "/swiftnav/rear/imu/data" topic. 
	* an EKF node that reads the IMU data produced by the complementary filter and fuses it with the odometry retrieved from the "/car_odometry" topic. It published the "/odometry/filtered" topic as an output.
	* the "/odometry/filtered" topic is used by a NAVSAT TRANSFORM NODE, that fuses the filtered odometry values along with the data retrieved by the "/swiftnav/rear/gps" topic. Even in this case we needed to remap the required gps topic from "/gps/fix" to "/swiftnav/rear/gps", that is the one available in our setting.  
  
**NOTE**: odometry is computed as in the **Ackerman** model, in the same way it is computed in the `odom_project` contained in this repo. 

```
  IMU                IMU                  IMU  
raw data  --->  complementary  --->  filtered data
                    filter               │
                                         │
                                         │
                                         V
	    Odometry  -------->  EKF  --->  filtered 
                                      odometry
                                         │
                                         │
                                         V
                       gps  -------->  navsat  --->  filtered
                       data             node           gps
```

## Parameters
For the navsat node we set the publish_filtered_gps parameters to true in order to have that topic published. We also set it to use the odometry yaw and to broadcast the utm transform. The magnetic declination has be set equal to Milan's one.  
The EKF node has been set to use 2D mode and to publish the tf transform. It retrieves the position, the yaw orientation and the yaw velocity from odometry, fusing it with the IMU data.  
For all the nodes we set the frequency to 30 Hz, as required.  
  
In the launch file, some "remap" tags have been used to remap the topics required by the nodes to those provided by the bag file.  
Other "rosparam" tags have been used to load the nodes' configurations from external files (we used a yaml file for each node).  

## Launch
Two simple launch files are provided: 
  * Simply use `roslaunch filters_project.launch` to run the node. Then if you want to reproduce the provided bag files, you can use `rosbag play <bag-files>`.
  * In case you want to run both the node and the bag files at the sime time, you can launch the `filters_project_with_bag.launch` file. Both bag files will be reproduced in this way.  

















