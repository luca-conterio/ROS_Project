# Autonomous Car GPS Position
Second Robotics Project - Politecnico di Milano 2018/2019  

## Overview
The aim of this project is to compute accurate and high frequency **GPS position** fusing odometry from wheels encoders and steering angle, IMU and GPS data from the Piksi Multi board.

## Project Structure
The project contains an `src` folder with the odometry node, a `launch` folder containing the launch file to run all the needed nodes and a `param` folder containing the configuration files for each one of the nodes.

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
A single node subscribes to the required topic (`/speedsteer`) to retrieve the car's velocity and steering angle, in order to compute its odometry. The odometry is then published on the `/car_odometry` topic. 
The node also published a tf transform from the `odom` frame to the `base_link` one.  
  
The project also instantiates:
  * a `complementary filter` node to filter IMU data read from `/swiftnav/rear/imu` topic. The used version is downloaded from official repositories. The `/imu` topic required by the filter has been remapped to `/swiftnav/rear/imu`. The filter then publishes `/swiftnav/rear/imu/data` topic. 
	* an `extended kalman filter` node that reads the IMU data produced by the complementary filter and fuses it with the odometry retrieved from the `/car_odometry` topic. It published the `/odometry/filtered` topic as an output.
	* the `/odometry/filtered` topic is used by a `navsat transform node`, that fuses the filtered odometry values along with the data retrieved by the `/swiftnav/rear/gps` topic. Even in this case the required gps topic from `/gps/fix` has been remapped to `/swiftnav/rear/gps`, that is the one available in this project's setting.  
  
**NOTE**: odometry is computed as in the **Ackerman** model, in the same way it is computed in the `odom_project` contained in this repo. 

```
   IMU    --->  Complementary  --->       IMU  
raw data           Filter            Filtered data
                                           │
                                           │
                                           │
                                           │
                         Extended          V
     Odometry  -------->  Kalman  --->  Filtered 
                          Filter        odometry
                                           │
                                           │
                                           V
                        GPS  -------->  navsat  --->  Filtered
                        data             node           gps
```

## Parameters
For the navsat node, the `publish_filtered_gps` parameter has been set to true in order to have that topic published. It has also been set to use the odometry yaw and to broadcast the `utm transform`. The magnetic declination has be set equal to Milan's one.  
The extended kalman filter node has been set to use 2D mode and to publish the `tf transform`. It retrieves the position, the yaw orientation and the yaw velocity from odometry, fusing it with the IMU data.  
For all the nodes we set the frequency to 30 Hz, as required.  
  
In the launch file, some `remap` tags have been used to remap the topics required by the nodes to those provided by the bag file.  
Other `rosparam` tags have been used to load the nodes' configurations from external files (we used a yaml file for each node).  

## Launch
Two simple launch files are provided: 
  * Simply use `roslaunch filters_project.launch` to run the node. Then if you want to reproduce the provided bag files, you can use `rosbag play <bag-files>`.
  * In case you want to run both the node and the bag files at the sime time, you can launch the `filters_project_with_bag.launch` file. Both bag files will be reproduced in this way.  

















