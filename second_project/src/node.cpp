
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

class pub_sub {

    private:
        ros::NodeHandle n; 
        ros::Subscriber sub;
        ros::Publisher pub;
        tf::TransformBroadcaster odom_broadcaster;
        
        
    public: pub_sub() {
        sub = n.subscribe("/speedsteer", 1, &pub_sub::callback, this);
        pub = n.advertise<nav_msgs::Odometry>("/car_odometry", 1);
    }

    void callback(const geometry_msgs::PointStamped& msg) {
        // compute the ackerman odometry
        double steering = msg.point.x;
        odometry_data.v = msg.point.y / 3.6; // convert velocity in m/s (it is given in km/h)
        current_time = Time::now();
        double wheels_angle = (steering / steering_ratio) * M_PI / 180; // convert to radians
        double dt = (current_time - last_time).toSec();
        odometry_data.vth = (odometry_data.v / front_rear_wheels_distance) * tan(wheels_angle);
        odometry_data.x = odometry_data.x + odometry_data.v * dt * cos(odometry_data.theta + (odometry_data.vth * dt) / 2);
        odometry_data.y = odometry_data.y + odometry_data.v * dt * sin(odometry_data.theta + (odometry_data.vth * dt) / 2);
        odometry_data.theta = odometry_data.theta + (odometry_data.vth * dt);
        odometry_data.current_time  = current_time;
        last_time = current_time;        
        ROS_INFO("%lf - %lf - %lf", wheels_angle, odometry_data.v, odometry_data.theta);

        nav_msgs::Odometry odom;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_data.theta);
        // publish the tf transform
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(odometry_data.x, odometry_data.y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, odometry_data.theta);
        transform.setRotation(q);
        odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
        // fill the odometry message with data
        odom.header.stamp = odometry_data.current_time;
        odom.header.frame_id = "odom";
        // set the position
        odom.pose.pose.position.x = odometry_data.x;
        odom.pose.pose.position.y = odometry_data.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        // set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = odometry_data.v * cos(odometry_data.theta);
        odom.twist.twist.linear.y = odometry_data.v * sin(odometry_data.theta);
        odom.twist.twist.angular.z = odometry_data.vth;
        // publish odom message
        pub.publish(odom);
    }
};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "subscribe_and_publish");
 	pub_sub my_pub_sub;
    ROS_INFO("Starting odometry node");
    last_time = Time::now();
 	ros::spin();
 	return 0;
}

