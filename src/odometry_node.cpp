#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "beast_msgs/Wheel.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <beast_odometry_publisher/odometry.h>
#include <sstream>

bool left_status_received = false;
auto left_angle = 0.0;
ros::Time left_last_status_time;
void leftWheelStatusCallback(const beast_msgs::Wheel::ConstPtr& left_wheel_status_msg) {
//    ROS_INFO("left angle: %0.4f", left_wheel_status_msg->angle);
    if(left_status_received){
        left_angle += left_wheel_status_msg->velocity * (left_wheel_status_msg->header.stamp - left_last_status_time).toSec();
    } else {
        left_status_received = true;
    }
    left_last_status_time = left_wheel_status_msg->header.stamp;
}

bool right_status_received = false;
auto right_angle = 0.0;
ros::Time right_last_status_time;
void rightWheelStatusCallback(const beast_msgs::Wheel::ConstPtr& right_wheel_status_msg) {
//    ROS_INFO("right angle: %0.4f", right_wheel_status_msg->angle);
    if(right_status_received){
        right_angle += right_wheel_status_msg->velocity * (right_wheel_status_msg->header.stamp - right_last_status_time).toSec();
    } else {
        right_status_received = true;
    }
    right_last_status_time = right_wheel_status_msg->header.stamp;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n("~");
    beast_odometry_publisher::Odometry odometry_;

    const std::string complete_ns = n.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    std::string name_= complete_ns.substr(id + 1);

    double publish_rate_ = 100.0;
    n.getParam("publish_rate", publish_rate_);
    ROS_INFO_STREAM_NAMED(name_, "Odometry will be published at " << publish_rate_ << "Hz.");

    /// Wheel separation, wrt the midpoint of the wheel width:
    double wheel_separation_ = 1.0;
    n.getParam("wheel_separation", wheel_separation_);
    ROS_INFO_STREAM_NAMED(name_, "wheel_separation set to " << wheel_separation_);

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_ = 1.0;
    n.getParam("wheel_radius", wheel_radius_);
    ROS_INFO_STREAM_NAMED(name_, "wheel_radius set to " << wheel_radius_);

    /// Wheel separation and radius calibration multipliers:
    double wheel_separation_multiplier_ = 1.0;
    n.getParam("wheel_separation_multiplier", wheel_separation_multiplier_);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation will be multiplied by " << wheel_separation_multiplier_ << ".");

    double left_wheel_radius_multiplier_ = 1.0;
    double right_wheel_radius_multiplier_ = 1.0;
    if (n.hasParam("wheel_radius_multiplier"))
    {
        double wheel_radius_multiplier;
        n.getParam("wheel_radius_multiplier", wheel_radius_multiplier);

        left_wheel_radius_multiplier_  = wheel_radius_multiplier;
        right_wheel_radius_multiplier_ = wheel_radius_multiplier;
    }
    else
    {
        n.getParam("left_wheel_radius_multiplier", left_wheel_radius_multiplier_);
        n.getParam("right_wheel_radius_multiplier", right_wheel_radius_multiplier_);
    }
    ROS_INFO_STREAM_NAMED(name_, "Left wheel radius will be multiplied by " << left_wheel_radius_multiplier_ << ".");
    ROS_INFO_STREAM_NAMED(name_, "Right wheel radius will be multiplied by " << right_wheel_radius_multiplier_ << ".");

    /// Frame to use for the robot base:
    std::string base_frame_id_ = "base_link";
    n.getParam("base_frame_id", base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    /// Frame to use for odometry and odom tf:
    std::string odom_frame_id_ = "odom";
    n.getParam("odom_frame_id", odom_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

    int velocity_rolling_window_size = 10;
    n.getParam("velocity_rolling_window_size", velocity_rolling_window_size);
    ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of " << velocity_rolling_window_size << ".");
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws  = wheel_separation_multiplier_   * wheel_separation_;
    const double lwr = left_wheel_radius_multiplier_  * wheel_radius_;
    const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;
    odometry_.setWheelParams(ws, lwr, rwr);
    ROS_INFO_STREAM_NAMED(name_,
                          "Odometry params"
                          << ": wheel separation " << ws
                          << ", left wheel radius "  << lwr
                          << ", right wheel radius " << rwr);

    //// end init

    ros::Publisher odom_pub_ = n.advertise<nav_msgs::Odometry>("/odom", 1000);
    static tf2_ros::TransformBroadcaster br;

    ros::Rate loop_rate(publish_rate_);
    odometry_.init(ros::Time::now());

    ros::Subscriber left_sub = n.subscribe("/beast_cart/left/wheel_status", 10, leftWheelStatusCallback);
    ros::Subscriber right_sub = n.subscribe("/beast_cart/right/wheel_status", 10, rightWheelStatusCallback);
    ros::Time previous_time = ros::Time::now();

    while (ros::ok()) {
        ros::Time time = ros::Time::now();
        if ((time - previous_time).toSec() < 0.0) {
            ROS_WARN_STREAM_NAMED(name_, "Resetting odometry due to backward jump in time");
            odometry_.init(time);
            left_angle = 0.0;
            left_status_received = false;
            right_angle = 0.0;
            right_status_received = false;
        }
        previous_time = time;

        // Estimate linear and angular velocity using joint information
//        ROS_INFO("previous odometry %0.4f %0.4f %0.4f", odometry_.getX(), odometry_.getY(), odometry_.getHeading());
        odometry_.update(left_angle, right_angle, time);
//        ROS_INFO("updated  odometry %0.4f %0.4f %0.4f", odometry_.getX(), odometry_.getY(), odometry_.getHeading());

        // Compute and store orientation info
        const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

        // Populate odom message and publish
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;
        odom_msg.pose.pose.position.x = odometry_.getX();
        odom_msg.pose.pose.position.y = odometry_.getY();
        odom_msg.pose.pose.orientation = orientation;
        odom_msg.twist.twist.linear.x  = odometry_.getLinear();
        odom_msg.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_.publish(odom_msg);

        // Publish tf from odom to robot base frame
        geometry_msgs::TransformStamped odom_frame;
        odom_frame.header.stamp = time;
        odom_frame.header.frame_id = odom_frame_id_;
        odom_frame.child_frame_id = base_frame_id_;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        br.sendTransform(odom_frame);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
