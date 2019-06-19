
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

using namespace ros;

// car baseline: distance between the wheels
static const double baseline = 1.3;
// distance between front and rear wheels of the car
static const double front_rear_wheels_distance = 1.765;
// steering ratio: for each 18° of the steer, the wheels rotate by 1°
static const double steering_ratio = 18;

// contains data computed via Ackerman odometry
typedef struct od_data {
    double x; 
    double y;
    double theta;
    double v;
    double vx;
    double vy;
    double vth;
    Time current_time;
} odometry_data_type;

odometry_data_type odometry_data;

Time current_time;
Time last_time;

// computes the Ackerman odometry, given the steering angle and the velocity of the car
void computeOdometry(double steering, double velocity) {
    current_time = Time::now();
    double wheels_angle = (steering / steering_ratio) * M_PI / 180; // convert to radians
    double dt = (current_time - last_time).toSec();
    double vth = (velocity / front_rear_wheels_distance) * tan(wheels_angle);
    odometry_data.v = velocity;
    odometry_data.x = odometry_data.x + odometry_data.v * dt * cos(odometry_data.theta + (odometry_data.vth * dt) / 2);
    odometry_data.y = odometry_data.y + odometry_data.v * dt * sin(odometry_data.theta + (odometry_data.vth * dt) / 2);
    odometry_data.theta = odometry_data.theta + (odometry_data.vth * dt);
    odometry_data.current_time  = current_time;
    last_time = current_time;
}

// publish odometry data on the two topics: /car_odometry and /car_odometry_with_type
void publishOdometry(ros::Publisher pub) {
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_data.theta);
    // fill the odometry message with data
    odom.header.stamp = odometry_data.current_time;
    odom.header.frame_id = "odom";
    // set the position
    odom.pose.pose.position.x = odometry_data.x;
    odom.pose.pose.position.y = odometry_data.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // set the velocity
    odom.child_frame_id = "car_frame";
    odom.twist.twist.linear.x = odometry_data.v * cos(odometry_data.theta);
    odom.twist.twist.linear.y = odometry_data.v * sin(odometry_data.theta);
    odom.twist.twist.angular.z = odometry_data.vth;
    // publish odom message
    pub.publish(odom);
}

void subscriberCallback(const geometry_msgs::PointStamped& msg) {
    //ROS_INFO("Received: %lf - %lf", msg.point.x, msg.point.y);
    computeOdometry(msg.point.x, msg.point.y);
    //ROS_INFO("Publishing");
    //publishOdometry(pub);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "odometry_node");

    ros::NodeHandle n;

    ros::Publisher odometry_pub = n.advertise<nav_msgs::Odometry>("/car_odometry", 50);
    ros::Subscriber sub = n.subscribe("/speedsteer", 1000, subscriberCallback);

    last_time = Time::now();

    ROS_INFO("Starting odometry node");

    ros::Rate rate(30.0);

    while(ros::ok()) {
        ros::spinOnce();
        publishOdometry(odometry_pub);
    }
    return 0;
}