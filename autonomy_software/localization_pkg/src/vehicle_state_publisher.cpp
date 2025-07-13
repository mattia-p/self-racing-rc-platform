/*
 * vehicle_state_publisher.cpp - Vehicle State Publisher
 *
 * Converts GPS coordinates to UTM and publishes vehicle state data
 * for navigation
 *
 */

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nmea_msgs/Gprmc.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure_pkg/vehicle_state_publisherConfig.h>

#define KNOTS_TO_MPS 0.5144

class VehicleStatePublisherNode {

private:
  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
  ros::Subscriber gps_sub_;
  int skip_message_count_;
  int last_msg_seq_;
  dynamic_reconfigure::Server<dynamic_reconfigure_pkg::vehicle_state_publisherConfig> server_;

public:
  VehicleStatePublisherNode() : nh_() {

    // Publishers and subscribers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 10);
    velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("current_velocity", 10);
    gps_sub_ = nh_.subscribe("gps_info", 10, &VehicleStatePublisherNode::gpsCallback, this);

    // Dynamic reconfigure setup
    server_.setCallback(boost::bind(&VehicleStatePublisherNode::dynamicReconfigureCallback, this, _1, _2));

    // Variable to store the last message sequence number
    last_msg_seq_ = -1;
  }

  void dynamicReconfigureCallback(dynamic_reconfigure_pkg::vehicle_state_publisherConfig &config, uint32_t level) {
    skip_message_count_ = config.jumping_message_nbr;
    ROS_INFO("Dynamic reconfigure call in vehicle_state_publisher.cpp");
  }

  // Move some of the logic to a separate function outside of the callback? Also for better unit testing?
  void gpsCallback(const nmea_msgs::Gprmc::ConstPtr &msg) {

    // For the first message, skip the checks
    if (last_msg_seq_ != -1) {

      if (msg->header.seq - last_msg_seq_ < 0) {
        ROS_WARN("The header sequence id has jumped back");
        ROS_WARN("Either you restarted the gps_publisher (or a rosbag play) or there is a problem");
      }
      // Skip message if we are below the skip count
      else if (msg->header.seq - last_msg_seq_ < 1 + skip_message_count_) {
        return;
      }
    }

    last_msg_seq_ = msg->header.seq;

    // Read subscriber data
    double latitude = msg->lat;
    double longitude = msg->lon;
    double speed_knots = msg->speed;
    double track_angle_deg = msg->track;

    // Convert to utm coordinates
    double x_utm, y_utm;
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, x_utm, y_utm);

    // Log all the ouputs
    ROS_INFO("UTM coordinates: x = %f, y = %f", x_utm, y_utm);
    ROS_INFO("Zone: %d, Northp: %d", zone, northp);

    // Build the /current_pose [PoseStamped] message
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = x_utm;
    pose_msg.pose.position.y = y_utm;
    pose_msg.pose.position.z = 0;

    // Yaw angle [radians]. Origin is horizontal axis. Counter clockwise
    double yaw_rad = (M_PI / 2) - track_angle_deg * M_PI / 180;

    // Convert orientation to quaternions
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_rad);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // Build the /current_velocity [TwistStamped] message
    geometry_msgs::TwistStamped velocity_msg;
    velocity_msg.header.stamp = ros::Time::now();
    velocity_msg.header.frame_id = "world";
    velocity_msg.twist.linear.x = speed_knots * KNOTS_TO_MPS;

    // Publish the /tf
    tf::TransformBroadcaster br;
    br.sendTransform(
        tf::StampedTransform(tf::Transform(q, tf::Vector3(x_utm, y_utm, 0)), ros::Time::now(), "world", "car"));

    // Publish the messages
    pose_pub_.publish(pose_msg);
    velocity_pub_.publish(velocity_msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vehicle_state_publisher");
  VehicleStatePublisherNode node;
  ros::spin();
}
