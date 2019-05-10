
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>
#include "first_project/floatStamped.h"




class PubSub {
  
  float64 speed_R;
  double speed_L;
  double steer;
  std_msgs::String odometry;
  //std_msgs::String speed_L;
	//std_msgs::String ...;

  private:
    ros::Subscriber sub_speed_R;
    ros::Subscriber sub_speed_L;
    ros::Subscriber sub_steer;
    ros::Publisher pub; 
    ros::Timer timer;

  public: PubSub(ros::NodeHandle n) {
    sub_speed_R = n.subscribe("/speed_R", 1000, &PubSub::callbackSpeedR, this);
    sub_speed_L = n.subscribe("/speed_L", 1000, &PubSub::callbackSpeedL, this);
    sub_steer = n.subscribe("/steer", 1000, &PubSub::callbackSteer, this);
    pub = n.advertise<std_msgs::String>("/odom", 1);
    //timer = n.createTimer(ros::Duration(1), &PubSub::timerCallback, this);
  }  

  void callbackSpeedR(const first_project::floatStamped::ConstPtr& msg) {
    speed_R = msg->data;
    ROS_INFO("Received speed_R: %lf", &speed_R);
  }

  void callbackSpeedL(const first_project::floatStamped::ConstPtr& msg) {
    speed_L = msg->data;
    ROS_INFO("Received speed_L: %lf", &speed_L);
  }

  void callbackSteer(const first_project::floatStamped::ConstPtr& msg) {
    steer = msg->data;
    ROS_INFO("Received steer: %lf", &steer);
  }

  void publishOdometry() {
    pub.publish(odometry);
  }

  void computeKinematicsOdometry() {
    ROS_INFO("Computing Kinematics Odometry...");
  }

  void computeAckermannOdometry() {
    ROS_INFO("Computing Ackermann Odometry...");
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
      ROS_INFO("Odometry type: %d", od_type);
      if (od_type == 0)
        pubsub->computeKinematicsOdometry();
      else if (od_type == 1)
        pubsub->computeAckermannOdometry();
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

  ROS_INFO("Spinning node");
  ros::Rate rate(1);

  while(ros::ok()) {
    //n.getParam("/odometry_type", odometry_type);
    computeOdometry(pubsub, odometry_type);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
