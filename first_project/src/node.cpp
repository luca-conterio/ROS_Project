
#include <ros/ros.h>
#include <math.h>
#include "std_msgs/String.h"
#include "first_project/floatStamped.h"
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

static const double baseline = 1.3;
static const double front_rear_wheels_distance = 1.765;
static const double steering_ratio = 18;

typedef struct s_data {
  double speed_R;
  double speed_L;
  double steer;
  double wheels_angle;
} sensors_data_type;

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

typedef message_filters::sync_policies::ApproximateTime<floatStamped, floatStamped, floatStamped> SyncPolicy;

class Odometry {
    private:
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
        //last_time = //Time::now();
    }

    public: void setOdometryType(int od_type) {
        odometry_type = od_type;
    }

    public: odometry_data_type compute(sensors_data_type s_data, Time current_time) {
        double dt = (current_time - last_time).toSec();
        odometry_data.v = (s_data.speed_R + s_data.speed_L) / 2.0;
        odometry_data.vx = odometry_data.v * cos(odometry_data.theta);
        odometry_data.vy = odometry_data.v * sin(odometry_data.theta);
        odometry_data.x = odometry_data.x + (odometry_data.vx * dt);
        odometry_data.y = odometry_data.y + (odometry_data.vy * dt);
        // differential drive kinematics
        if (odometry_type == 0) {           
            ROS_INFO("Odometry type: %d (%s)", odometry_type, "Differential Drive");
            odometry_data.vth = (s_data.speed_R - s_data.speed_L) / baseline;
        }
        // ackermann kinematics
        else if (odometry_type == 1) {     
            ROS_INFO("Odometry type: %d (%s)", odometry_type, "Ackermann");
            odometry_data.vth = (odometry_data.v / front_rear_wheels_distance) * tan(s_data.wheels_angle);
        }
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

void publishOdometry(ros::Publisher pub, tf::TransformBroadcaster odom_broadcaster, odometry_data_type od_data) {
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(od_data.theta);
    //publish the transform over tf
    /* geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = od_data.current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = od_data.x;
    odom_trans.transform.translation.y = od_data.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster.sendTransform(odom_trans); */

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(od_data.x, od_data.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, od_data.theta);
    transform.setRotation(q);
    odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odometry_node"));
    

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = od_data.current_time;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x = od_data.x;
    odom.pose.pose.position.y = od_data.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = od_data.vx;
    odom.twist.twist.linear.y = od_data.vy;
    odom.twist.twist.angular.z = od_data.vth;

    ROS_INFO("\n\n ------- Publishing ------- \n x: %lf \n y: %lf \n theta: %lf \n vx: %lf \n vy: %lf \n vth: %lf \n --------------------------\n", 
                                                        od_data.x, od_data.y, od_data.theta, od_data.vx, od_data.vy, od_data.vth);
    pub.publish(odom);
}

void publishOdometryType(ros::Publisher pub, Odometry *odometry) {
    std_msgs::String odometry_type_msg;
        if (odometry->getOdometryType() == 0)
            odometry_type_msg.data = "Differential Drive";
        else if (odometry->getOdometryType() == 1)
            odometry_type_msg.data = "Ackermann";
        pub.publish(odometry_type_msg);
}

void synchronizedCallback(Odometry *odometry, ros::Publisher pub, 
                                tf::TransformBroadcaster odom_broadcaster, const floatStampedConstPtr& msg1, 
                                const floatStampedConstPtr& msg2, const floatStampedConstPtr& msg3) {
    sensors_data_type s_data;
    s_data.speed_R = msg1->data;    
    s_data.speed_L = msg2->data;
    s_data.steer = msg3->data;
    double angle_degrees = s_data.steer / steering_ratio;
    s_data.wheels_angle = angle_degrees * M_PI / 180; // 0.0174533 ---> convert to radians
    ROS_INFO("\n\n ------- Received ------- \n speed_R: %lf \n speed_L: %lf \n steer: %lf \n wheels_angle: %lf \n ------------------------\n", 
                                        s_data.speed_R, s_data.speed_L, s_data.steer, s_data.wheels_angle);
    odometry_data_type odometry_data = odometry->compute(s_data, Time::now());
    publishOdometry(pub, odom_broadcaster, odometry_data);
}                

void parameterServerCallback(ros::NodeHandle n, Odometry *odometry, parametersConfig config) {
    odometry->setOdometryType(config.odometry_type);
    if (config.reset_position == true) {
            odometry->setPosition(config.x_pos, config.y_pos);
        n.setParam("reset_position", false);
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "odometry_node");

    Odometry *odometry = new Odometry();

    ros::NodeHandle n;

    ros::Publisher odometry_pub = n.advertise<nav_msgs::Odometry>("/car_odometry", 50);
    ros::Publisher odometry_type_pub = n.advertise<std_msgs::String>("/car_odometry_type", 50);
    tf::TransformBroadcaster odom_broadcaster;

    message_filters::Subscriber<floatStamped> sub_speed_R(n, "/speedR_stamped", 1);
    message_filters::Subscriber<floatStamped> sub_speed_L(n, "/speedL_stamped", 1);
    message_filters::Subscriber<floatStamped> sub_steer(n, "/steer_stamped", 1);
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_speed_R, sub_speed_L, sub_steer);
    sync.registerCallback(boost::bind(&synchronizedCallback, odometry, odometry_pub, odom_broadcaster, _1, _2, _3));

    dynamic_reconfigure::Server<parametersConfig> parameterServer;
    parameterServer.setCallback(boost::bind(&parameterServerCallback, n, odometry, _1));

    ROS_INFO("Spinning node");
    Rate rate(1.0);

    while(ros::ok()) {
        ros::spinOnce();
        publishOdometryType(odometry_type_pub, odometry);
        rate.sleep();
    }

    return 0;
}