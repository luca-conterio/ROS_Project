
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "first_project/floatStamped.h"
#include "first_project/odometryMessage.h"
#include "message_filters/subscriber.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

using namespace ros;
using namespace first_project;
using namespace message_filters;


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
} odometry_data_type;

typedef message_filters::sync_policies::ApproximateTime<floatStamped, floatStamped, floatStamped> SyncPolicy;


class Odometry {
    private:
        double B = 1.30;
        double L = 1.765;
        int STEERING_RATIO = 18;
        double odometry;
        int odometry_type;
        odometry_data_type odometry_data;

    public: Odometry() {
        odometry_type = 0;
        odometry_data.x = 0.0;
        odometry_data.y = 0.0;
        odometry_data.theta = 0.0;
    }

    public: void setOdometryType(int od_type) {
        odometry_type = od_type;
    }

    public: odometry_data_type compute(sensors_data_type sensors_data) {
        // differential drive kinematics
        if (odometry_type == 0) {           
            ROS_INFO("Odometry type: %d (%s)", odometry_type, "Differential_Drive");
        }
        // ackerman kinematics
        else if (odometry_type == 1) {     
            ROS_INFO("Odometry type: %d (%s)", odometry_type, "Ackerman");
        }

        return odometry_data;
    }

    public: void resetPosition() {
        ROS_INFO(" Resetting position to (0,0)");
        odometry_data.x = 0;
        odometry_data.y = 0;
    }

    public: void setPosition(int x_pos, int y_pos) {
        ROS_INFO(" Setting position to (%d,%d)", x_pos, y_pos);
        odometry_data.x = x_pos;
        odometry_data.y = y_pos;
    }
};


void synchronizedCallback(sensors_data_type *s_data, const floatStampedConstPtr& msg1, const floatStampedConstPtr& msg2, const floatStampedConstPtr& msg3) {
    s_data->speed_R = msg1->data;    
    s_data->speed_L = msg2->data;
    s_data->steer = msg3->data;
    s_data->wheels_angle = s_data->steer / 18;
    ROS_INFO("\n\n ------- Received ------- \n speed_R: %lf \n speed_L: %lf \n steer: %lf \n wheels_angle: %lf \n ------------------------\n", s_data->speed_R, s_data->speed_L, s_data->steer, s_data->wheels_angle);
}                

void publishOdometry(ros::Publisher pub, odometry_data_type od_data) {
    odometryMessage msg;
    msg.x = od_data.x;
    msg.y = od_data.y;
    msg.theta = od_data.theta;
    ROS_INFO("\n\n ------- Publishing ------- \n x: %lf \n y: %lf \n theta: %lf \n ------------------------\n", msg.x, msg.y, msg.theta);
    pub.publish(msg);
}

void parameterServerCallback(Odometry *odometry, parametersConfig config) {
    odometry->setOdometryType(config.odometry_type);
    if (config.reset_position == true)
        if (config.x_pos == 0 and config.y_pos == 0)
            odometry->resetPosition();
        else 
            odometry->setPosition(config.x_pos, config.y_pos);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "main_node");

    Odometry *odometry = new Odometry();
    sensors_data_type sensors_data;
    odometry_data_type odometry_data;
    odometryMessage odometry_msg;

    ros::NodeHandle n; 
    ros::Publisher pub = n.advertise<odometryMessage>("/odom", 50);
    message_filters::Subscriber<floatStamped> sub_speed_R(n, "/speedR_stamped", 1);
    message_filters::Subscriber<floatStamped> sub_speed_L(n, "/speedL_stamped", 1);
    message_filters::Subscriber<floatStamped> sub_steer(n, "/steer_stamped", 1);
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_speed_R, sub_speed_L, sub_steer);
    sync.registerCallback(boost::bind(&synchronizedCallback, &sensors_data, _1, _2, _3));
    //PubSub *pubsub = new PubSub(n, /*sub_speed_R, sub_speed_L, sub_steer,*/ sync);

    dynamic_reconfigure::Server<parametersConfig> parameterServer;
    parameterServer.setCallback(boost::bind(&parameterServerCallback, odometry, _1));

    ROS_INFO("Spinning node");
    Time current_time, last_time;
    current_time = Time::now();
    last_time = Time::now();

    Rate rate(1.0);

    while(ros::ok()) {

        spinOnce();
        current_time = Time::now();

        /*
            COMPUTATION
        */
        odometry_data = odometry->compute(sensors_data);
        publishOdometry(pub, odometry_data);

        last_time = current_time;
        rate.sleep();
    }

    return 0;
}