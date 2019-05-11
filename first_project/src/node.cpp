
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>
#include "first_project/floatStamped.h"




class PubSub {
  
  double speed_R = 0.0;
  double speed_L = 0.0;
  double steer = 0.0;

  private:
    ros::Subscriber sub_speed_R;
    ros::Subscriber sub_speed_L;
    ros::Subscriber sub_steer;
    ros::Publisher pub; 
    //ros::Timer timer;

  public: PubSub(ros::NodeHandle n) {
    sub_speed_R = n.subscribe("/speed_R", 1000, &PubSub::callbackSpeedR, this);
    sub_speed_L = n.subscribe("/speed_L", 1000, &PubSub::callbackSpeedL, this);
    sub_steer = n.subscribe("/steer", 1000, &PubSub::callbackSteer, this);
    pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
    //timer = n.createTimer(ros::Duration(1), &PubSub::timerCallback, this);
  }  

  void callbackSpeedR(const std_msgs::Float64::ConstPtr& msg) {
    speed_R = msg->data;
    //ROS_INFO("Received speed_R: %lf", speed_R);
  }

  void callbackSpeedL(const std_msgs::Float64::ConstPtr& msg) {
    speed_L = msg->data;
    //ROS_INFO("Received speed_L: %lf", speed_L);
  }

  void callbackSteer(const std_msgs::Float64::ConstPtr& msg) {
    steer = msg->data;
    //ROS_INFO("Received steer: %lf", steer);
  }

  void publishOdometry(double x, double y, double theta, double vx, double vy, double w, ros::Time current_time) {
    ROS_INFO("Publishing: %lf - %lf - %lf", x, y, theta);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    nav_msgs::Odometry odom;

    // set the header
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = w;

    //publish the message
    pub.publish(odom);
  }

  void computeKinematicsOdometry() {
    ROS_INFO("Computing Kinematics Odometry...");
  }

  void computeAckermannOdometry() {
    ROS_INFO("Computing Ackermann Odometry...");
  }

  double getSpeedR() {
    return speed_R;
  }

  double getSpeedL() {
    return speed_L;
  }

  double getSteer() {
    return steer;
  }

  /*void timerCallback(const ros::TimerEvent&) {
    //pub.publish(odometry);
    //ROS_INFO("Published odometry value");
  }

  void setOdometryType(first_project::parametersConfig &config) {
    odometry_type = config.odometry_type.c_str();
  }
  */
};



void parametersServerCallback(int *od_type, first_project::parametersConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d", config.odometry_type);
  *od_type = config.odometry_type;
  //ROS_INFO ("%d", level);
}

void computeOdometry(PubSub *pubsub, int od_type) {
      if (od_type == 0) {
        ROS_INFO("Odometry type: %d (%s)", od_type, "kinematics");
        pubsub->computeKinematicsOdometry();
      }
      else if (od_type == 1) {
        ROS_INFO("Odometry type: %d (%s)", od_type, "akermann");
        pubsub->computeAckermannOdometry();
      }
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "main");

  //rosbag::Bag bag;
  //bag.open("bag_1.bag", rosbag::bagmode::Read);

  int odometry_type;
  ros::NodeHandle n; 
  PubSub *pubsub = new PubSub(n);
  dynamic_reconfigure::Server<first_project::parametersConfig> parameterServer;
  dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType callbackFunction;
  callbackFunction = boost::bind(&parametersServerCallback, &odometry_type, _1, _2);
  parameterServer.setCallback(callbackFunction);

  double x = 0.0;
  double y = 0.0;
  double angle = 0.0;

  ROS_INFO("Spinning node");
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate rate(1.0);

  while(n.ok()) {
    
    ros::spinOnce();
    current_time = ros::Time::now();

    computeOdometry(pubsub, odometry_type);

    double speedR = pubsub->getSpeedR();
    double speedL = pubsub->getSpeedL();
    double angle = pubsub->getSteer();

    ROS_INFO("Received speed_R: %lf", speedR);
    ROS_INFO("Received speed_L: %lf", speedL);
    ROS_INFO("Received steer: %lf", angle);

    double w = (speedR - speedL) / 2.0;
    double v = (speedR + speedL) / 2.0;
    double vx = v * cos(angle);
    double vy = v * sin(angle);
    double dt = (current_time - last_time).toSec();

    double delta_x = vx * dt;
    double delta_y = vy * dt;
    double delta_angle = w * dt;

    x += delta_x;
    y += delta_y;
    angle += delta_angle;

    pubsub->publishOdometry(x, y, angle, vx, vy, w, current_time);

    last_time = current_time;
    rate.sleep();
  }

  return 0;
}
