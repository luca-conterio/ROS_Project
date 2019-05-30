# Autonomous Car Odometry 
First Robotics Project - Politecnico di Milano 2018/2019  
  
## Overview
The goal of this simple project is to retrieve data from a bag file recorded by sensors of an autonomous car and to compute its odometry, both using **Differential Drive** kinematics and **Ackerman** model.  
It uses **ROS (Robot Operating System)** to retrieve the data, to compute the odometry and to publish the results.

## Project Structure
```
first_project  
   ├── cfg  
   │   └── parameters.cfg  
   ├── CMakeLists.txt. 
   ├── launch 
   │   ├── first_project.launch 
   │   ├── bag_1.bag
   │   └── bag_2.bag
   ├── msg  
   │   ├── floatStamped.msg  
   │   └── odometryMessage.msg  
   ├── package.xml  
   └── src  
       └── node.cpp  
```
## Car Parameters
  * **Rear wheels baseline**: `130 cm`.
  * **Distance between front and rear wheels**: `176.5 cm`.
  * the car has a **steering ratio** equal to `18`.

## Nodes
A single node subscribes to the needed topics (`/speedR_stamped`, `/speedL_stamped`, `/steer_stamped`).  
It also publishes odometry data after computation. Odometry data is published with a standard `nav_msgs::Odometry` message (useful to be plotted for visualization), a custom `first_project::odometryMessage`, that contains odometry data along with the odometry type (`Differential_Drive` or `Ackerman`), and as tf transform.

## Launch
Two simple launch files are provided: 
  * Simply use `roslaunch first_project.launch` to run the node. Then if you want to reproduce the provided bag files, you can use `rosbag play <bag-files>`.
  * In case you want to run both the node and the bag file at the sime time, you can launch the `first_project_with_bag.launch` file.  

## Messages and Topics
Data from the bag file is published onto three topics, that are used for the computation:
  * `/speedR_stamped`: here the speed of the right wheel is published (in `m/s`).
  * `/speedL_stamped`: here the speed of the left wheel is published (in `m/s`).
  * `/steer_stamped`: here the steering angle is published (in degrees).  
  
Since the messages published on these three topics contain also a timestamp, the node uses `message_filters::sync_policies::ApproximateTime` policy to synchronize the received messages.  

The same data, without timestamps, is published also onto other three topics: `/speed_R`, `/speed_L` and `/steer` (standard `float64` messages).  
  
In the `msg` folder two custom message definitions are contained:
  * `floatStamped.msg`: used to subscribe to car sensors data (through the above described topics), since bag files data is formatted as this message type.
  * `odometryMessage.msg`: used by the node to publish odometry data along with the odometry type used in the computation (that can be either `Differential_Drive` or `Ackerman`). So it contains a `nav_msgs::Odometry` field called `odometry` and a string field called `source_type`.  
  
Besides these two custom message types, the node uses `nav_msgs::Odometry` to publish odometry data in a standard way, so that it is simpler to plot those data for trajectory visualization.  
  
As a result odometry data messages are published onto two topics:
  * `nav_msgs::Odometry` messages are published onto `/car_odometry` topic.
  * `first_project::odometryMessage` messages are published onto `/car_odometry_with_type` topic (custom message type).

#### Topics Graph
```
                        _________________                                 _______________
                       /                 \                               /               \
                ----> |  /speedR_stamped  | ----                  ----> |  /car_odometry  |
               /       \_________________/      \                /       \_______________/
   __________ /         _________________        \   __________ /
  /          \         /                 \        > /          \
 |  /rosbag   | ----> |  /speedL_stamped  | -----> |   /node    |
  \__________/         \_________________/        > \__________/
              \         _________________        /              \         _________________________
               \       /                 \      /                \       /                         \
                ----> |      /steer       | ----                  ----> |  /car_odometry_with_type  |
                       \_________________/                               \_________________________/

```

#### Tf_tree Structure
Odometry data is published also as a tf transform through a `tf::TransformBroadcaster`.  
This is the tf tree structure, where `odom` is the parent frame, while `car_frame` is the robot's frame:  
```
    __________
   /          \
  |    odom    |
   \__________/
        |
        |
        |
        V
    __________
   /          \
  | car_frame  |
   \__________/

```

## Odometry Computation
To have a better approximation of the real values, the odometry is computed according to the Rugge-Kutta integration, instead of the Euler's one:
```
x_k+1 = x_k + v * dt * cos(theta_k + (w_k * dt) / 2)  
y_k+1 = y_k + v * dt * sin(theta_k + (w_k * dt) / 2)  
theta_k+1 = theta_k + (w_k * dt)  
dt = T_k+1 - T_k
```
where `k` indicates the current iteration and `k+1` the following one, `dt` the elapsed time between the two consecutive iterations and `w` the angular velocity.
The angular velocity is instead computed according to the selected odometry type (differential drive or Ackerman).

## Configuration and Parameters
The `cfg` directory contains a `parameters.cfg` file with the specification of a `ParameterGenerator`.  
It is used in the node to instantiate a `dynamice_reconfigure::Server` to allow dynamic reconfiguration of the robot position `(x,y)`.  
Four parameters have been defined:
  * `x_pos`: it respresents the robot's x coordinate.
  * `y_pos`: it represents the robot's y coordinate.
  * `reset_position`: it is used to enable the user to set the robot's position to a desired value `(x,y)`.
  * `odometry_type`: it is used to change the type of odometry (`0` for `Differential_Drive` and `1` for `Ackerman`).

#### Dynamic Reconfigure
The `dynamic_reconfigure` package is used to set/reset the robot position to an arbitrary value `(x,y)`. 
The parameters to be modified to set the position are: `x_pos` and `y_pos`.  
After you have set the position to the requested one, if you set the `reset_position` parameter to `true` the `dynamic_reconfigure::Server` callback function is called, setting the robot position to the specified one.  
  
We decided not to reset the orientation to a specified value nor to 0 when a dynamic reconfiguration of robot position is performed, so the node continues computing the odometry based on the previously computed orientation, also after a reset of its position `(x,y).`  
  
**NOTE**: the reconfiguration is performed only if `reset_position` parameter is set to `true`. This paramter acts as a sort of "button" (in `rqt_reconfigure` it appears as a checkbox) that modifies the current robot position. It is useful to set `x_pos` and `y_pos` to the desired ones, then set `reset_position` to `true` to update the robot position and put `reset_position` to `false` again before setting the position to a new value, repeating the procedure. This allows you to update the position "in one shot".  
  
For what concerns changing the odometry computation type, it can be done simple changing the `odometry_type` parameter value (dropdown menu in `rqt_reconfigure`), selecting the desired odometry type.  
  
**NOTE**: of course, if you change the odometry type while `reset_position` is set to `true`, the callback function for the `ParameterServer` will be called and also the position will be updated to the value of `x_pos` and `y_pos`. It is more usefule to change the odometry type when `reset_position` is set to `false`, so that only the odometry type will be changed.

