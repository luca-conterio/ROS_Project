# Autonomous Car Odometry 
First Robotics Project - Politecnico di Milano 2018/2019  
  
### Overview
The goal of this simple project is to retrieve data from a bag recorded by sensors of an autonomous car and to compute its odometry, both using **Differential Drive** kinematics and **Ackermann** model.  
It uses **ROS (Robot Operating System)** to retrieve the data, to compute the odometry and to publish the results.

### Project Structure
```
first_project  
   ├── cfg  
   │   └── parameters.cfg  
   ├── CMakeLists.txt. 
   ├── launch  
   │   └── first_project.launch  
   ├── msg  
   │   ├── floatStamped.msg  
   │   └── odometryMessage.msg  
   ├── package.xml  
   └── src  
       └── node.cpp  
```
### Car Parameters:
  * **Rear wheels baseline**: `130 cm`.
  * **Distance between front and rear wheels**: `176.5 cm`.
  * the car has a **steering ratio** equal to `18`.

### Nodes
A single node subscribes to the needed topics (`/speedR_stamped`, `/speedL_stamped`, `/steer_stamped`).  
It also publishes odometry data after computation. Odometry data is published with a standard `nav_msgs::Odometry` message (useful to be plotted for visualization), a custom `first_project::odometryMessage`, that contains odometry data along with the odometry type (`Differential_Drive` or `Ackermann`), and as tf transform.

### Launch
A simple launch file is provided to start the ros node.  
Simply use `roslaunch first_project.launch` to run it.

### Messages and Topics
Data from the bag file are published on three topics:
  * `/speedR_stamped`: here the speed of the right wheel is published (in `m/s`).
  * `/speedL_stamped`: here the speed of the left wheel is published (in `m/s`).
  * `/steer_stamped`: here the steering angle is published (in degrees).
  
In the `msg` folder two custom message definitions are contained:
  * `floatStamped.msg`: used to subscribe to car sensors data, since those data are formatted as this type of message.
  * `odometryMessage.msg`: used by the node to publish odometry data along with the odometry type used in the computation (that can be either "Differential_Drive" or "Ackermann"). So it contains a nav_msgs::Odometry field called "odometry" and a string field called "source_type".  
  
Besides these two custom message types, the node uses nav_msgs::Odometry to publish odometry data in a standard way, so that it is simpler to plot those data for trajectory visualization.  
  
As a result odometry data messages are published onto two topics:
  * `nav_msgs::Odometry` messages are published onto `/car_odometry` topic.
  * `first_project::odometryMessage` messages are published onto `/car_odometry_with_type` topic (custom message type).

### Configuration and Parameters
The `cfg` directory contains a `parameters.cfg` file with the specification of a `ParameterGenerator`.  
It is used in the node to instantiate a `dynamice_reconfigure::Server` to allow dynamic reconfiguration of the robot position (x,y).  
Three parameters have been defined:
  * `x_pos`: it respresents the robot's x coordinate.
  * `y_pos`: it represents the robot's y coordinate.
  * `reset_position`: it is used to enable the user to set the robot's position to a desired value (x,y).

### Dynamic Reconfigure
It is used to set/reset the robot position to an arbitrary position (x,y). 
The parameters to be modified to set the position are: `x_pos` and `y_pos`. After you have set the position to the requested one, if you set the `reset_position` parameter to `true` the `dynamic_reconfigure::Server` callback function is called, setting the robot position to the specified one.  
  
We decided not to reset the orientation to a specified value nor to 0 when a dynamic reconfiguration of robot position is performed, so the node continues computing the odometry based on the previoused computed orientation, also after a reset of its position (x,y).  
  
**NOTE**: the reconfiguration is performed only if `reset_position` parameter is set to `true`, so it acts as a sort of button (even if in `rqt_reconfigure` it appears as a checkbox) that modifies the current robot position. It is useful to set `x_pos` and `y_pos` to the desired ones, then set `reset_position` to `true` to update the robot position and put `reset_position` to `false` again before setting the position to a new value, repeating the procedure.

### Tf Tree Structure
Odometry data is published also as a tf transform through a `tf::TransformBroadcaster`.  
This is the tf tree structure, where `odom` is the parent frame, while `car` is the robot's frame:  
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
  |    car     |
   \__________/
```
