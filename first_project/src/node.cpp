
#include <ros/ros.h>
#include <math.h>
#include "std_msgs/String.h"
#include "first_project/floatStamped.h"
#include "first_project/odometryMessage.h"
#include <nav_msgs/Odometry.h>
#include "message_filters/subscriber.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <first_project/parametersConfig.h>

using namespace ros;
using namespace first_project;
using namespace message_filters;

// car baseline: distance between the wheels
static const double baseline = 1.3;
// distance between front and rear wheels of the car
static const double front_rear_wheels_distance = 1.765;
// steering ratio: for each 18° of the steer, the wheels rotate by 1°
static const double steering_ratio = 18;

// contains data from sensors (bag file topics)
typedef struct s_data {
  double speed_R;
  double speed_L;
  double steer;
  double wheels_angle;
} sensors_data_type;

// contains data to be published as odometry message
typedef struct od_data {
    char source_type[20];
    double x; 
    double y;
    double theta;
    double v;
    double vx;
    double vy;
    double vth;
    Time current_time;
} odometry_data_type;

// policy used to synch data retrieved from the bag file topics
typedef message_filters::sync_policies::ApproximateTime<floatStamped, floatStamped, floatStamped> SyncPolicy;

// class used to computed odometry and store data (position, velocities, etc...)
class Odometry {
    private:
        // used to distinguish differential drive and ackerman odometry
        int odometry_type;
        odometry_data_type odometry_data;
        Time last_time;

    public: Odometry() {
        odometry_type = 0;
        odometry_data.x = 0.0;
        odometry_data.y = 0.0;
        odometry_data.theta = 0.0;
        odometry_data.v = 0.0;
        odometry_data.vx = 0.0;
        odometry_data.vy = 0.0;
        odometry_data.vth = 0.0;
    }

    public: void setOdometryType(int od_type) {
        odometry_type = od_type;
    }

    public: odometry_data_type compute(sensors_data_type s_data, Time current_time) {
        double dt = (current_time - last_time).toSec();
        odometry_data.v = (s_data.speed_R + s_data.speed_L) / 2.0;
        // differential drive kinematics
        if (odometry_type == 0) {           
            ROS_INFO("Odometry type: %d (%s)", odometry_type, "Differential Drive");
            strcpy(odometry_data.source_type, "Differential_Drive");
            odometry_data.vth = (s_data.speed_R - s_data.speed_L) / baseline;
        }
        // ackerman kinematics
        else if (odometry_type == 1) {     
            ROS_INFO("Odometry type: %d (%s)", odometry_type, "ackerman");
            strcpy(odometry_data.source_type, "ackerman");
            odometry_data.vth = (odometry_data.v / front_rear_wheels_distance) * tan(s_data.wheels_angle);
        }

        odometry_data.x = odometry_data.x + odometry_data.v * dt * cos(odometry_data.theta + (odometry_data.vth * dt) / 2);
        odometry_data.y = odometry_data.y + odometry_data.v * dt * sin(odometry_data.theta + (odometry_data.vth * dt) / 2);
        //odometry_data.x = odometry_data.x + (odometry_data.vx * dt);
        //odometry_data.y = odometry_data.y + (odometry_data.vy * dt);
        odometry_data.theta = odometry_data.theta + (odometry_data.vth * dt);
        odometry_data.current_time  = current_time;
        last_time = current_time;
        return odometry_data;
    }

    public: void setPosition(int x_pos, int y_pos) {
        ROS_INFO(" Setting position to (%d,%d)", x_pos, y_pos);
        odometry_data.x = x_pos;
        odometry_data.y = y_pos;
    }

    public: int getOdometryType() {
        return odometry_type;
    }
};

//#########################################################################################################

// publish odometry data on the two topics: /car_odometry and /car_odometry_with_type
void publishOdometry(ros::Publisher pub, ros::Publisher pub_with_type, 
                        tf::TransformBroadcaster odom_broadcaster, odometry_data_type od_data) {
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(od_data.theta);

    // publish odometry data as tf transform
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(od_data.x, od_data.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, od_data.theta);
    transform.setRotation(q);
    odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "car_frame"));
    
    // fill the odometry message with data
    odom.header.stamp = od_data.current_time;
    odom.header.frame_id = "odom";
    // set the position
    odom.pose.pose.position.x = od_data.x;
    odom.pose.pose.position.y = od_data.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = od_data.vx;
    odom.twist.twist.linear.y = od_data.vy;
    odom.twist.twist.angular.z = od_data.vth;

    ROS_INFO("\n\n ------- Publishing ------- \n x: %lf \n y: %lf \n theta: %lf \n vx: %lf \n vy: %lf \n vth: %lf \n --------------------------\n", 
                                                        od_data.x, od_data.y, od_data.theta, od_data.vx, od_data.vy, od_data.vth);
    // publish a nav_msgs::Odometry message
    pub.publish(odom);

    // publish custom message (first_project::odometryMessage) with odometry data and the odometry type
    odometryMessage odom_with_type;
    odom_with_type.source_type = od_data.source_type;
    odom_with_type.odometry = odom;
    pub_with_type.publish(odom_with_type);
}

// synchronize data retreived from bag file topics
void synchronizedCallback(Odometry *odometry, ros::Publisher pub, ros::Publisher pub_with_type,
                                tf::TransformBroadcaster odom_broadcaster, const floatStampedConstPtr& msg1, 
                                const floatStampedConstPtr& msg2, const floatStampedConstPtr& msg3) {
    sensors_data_type s_data;
    s_data.speed_R = msg1->data;    
    s_data.speed_L = msg2->data;
    s_data.steer = msg3->data;
    // get the wheels angle from the steering angle
    double angle_degrees = s_data.steer / steering_ratio;
    s_data.wheels_angle = angle_degrees * M_PI / 180; // 0.0174533 ---> convert to radians
    ROS_INFO("\n\n ------- Received ------- \n speed_R: %lf \n speed_L: %lf \n steer: %lf \n wheels_angle: %lf \n ------------------------\n", 
                                        s_data.speed_R, s_data.speed_L, s_data.steer, s_data.wheels_angle);
    odometry_data_type odometry_data = odometry->compute(s_data, Time::now());
    publishOdometry(pub, pub_with_type, odom_broadcaster, odometry_data);
}                

// reset/set parameters with dynamic_reconfigure (x_pos and y_pos params)
void parameterServerCallback(ros::NodeHandle n, Odometry *odometry, parametersConfig config) {
    odometry->setOdometryType(config.odometry_type);
    if (config.reset_position == true) {
        odometry->setPosition(config.x_pos, config.y_pos);
        //n.setParam("reset_position", false);
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "odometry_node");

    Odometry *odometry = new Odometry();

    ros::NodeHandle n;

    // create 2 publisher: one for odometry standard message and one 
    // for the custom message (odometryMessage) that includes the odometry type
    ros::Publisher odometry_pub = n.advertise<nav_msgs::Odometry>("/car_odometry", 50);
    ros::Publisher odometry_with_type_pub = n.advertise<odometryMessage>("/car_odometry_with_type", 50);
    // create a broadcaster to publish odometry data over tf transform
    tf::TransformBroadcaster odom_broadcaster;

    // subscribers for all the topics needed (right and left wheels and the steering angle)
    message_filters::Subscriber<floatStamped> sub_speed_R(n, "/speedR_stamped", 1);
    message_filters::Subscriber<floatStamped> sub_speed_L(n, "/speedL_stamped", 1);
    message_filters::Subscriber<floatStamped> sub_steer(n, "/steer_stamped", 1);
    // synchronize the data retrieved from the topics
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_speed_R, sub_speed_L, sub_steer);
    sync.registerCallback(boost::bind(&synchronizedCallback, odometry, odometry_pub, odometry_with_type_pub, odom_broadcaster, _1, _2, _3));

    // create a parameter server, allowing dynamic_reconfigure of parameters
    dynamic_reconfigure::Server<parametersConfig> parameterServer;
    parameterServer.setCallback(boost::bind(&parameterServerCallback, n, odometry, _1));

    ROS_INFO("Spinning node");
    //Rate rate(10.0);
    ros::spin();

    return 0;
}