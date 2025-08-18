#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "icp/eigen_kdtree.h"
#include "localizer2d.h"
#include "map.h"
#include "ros_bridge.h"

// Map callback definition
void callback_map(const nav_msgs::OccupancyGridConstPtr&);
// Initial pose callback definition
void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
// Scan callback definition
void callback_scan(const sensor_msgs::LaserScanConstPtr&);

std::shared_ptr<Map> map_ptr = nullptr;
ros::Publisher pub_scan, pub_odom;
ros::Subscriber map_sub, initial_pos_sub, base_scan_sub;

Localizer2D localizer;

int main(int argc, char** argv) {
  // Initialize ROS system
  ros::init(argc, argv, "localizer_node");
  // TODO

  // Create a NodeHandle to manage the node.
  // The namespace of the node is set to global
  ros::NodeHandle nh("/node_localizer");

  // Create shared pointer for the Map object
  // TODO
  map_ptr = std::make_shared<Map>();

  //
  
  // * Subscribe to the topics:
  // * /map [nav_msgs::OccupancyGrid]
  map_sub = nh.subscribe("/map", 10, callback_map);

  // * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
  initial_pos_sub = nh.subscribe("/initialpose", 10, callback_initialpose);
  // * /base_scan [sensor_msgs::LaserScan]
  base_scan_sub = nh.subscribe("/base_scan", 10, callback_scan);
  // * and assign the correct callbacks
  // *
  // * Advertise the following topic:
  // * /odom_out [nav_msgs::Odometry]
  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_out", 10);
   
  // TODO

  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);

  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();

  return 0;
}

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg_) {
  // If the internal map is not initialized, load the incoming occupancyGrid and
  // set the localizer map accordingly
  // Remember to load the map only once during the execution of the map.

  // TODO

  if(!map_ptr->initialized()){

    ROS_INFO("Map Initialized: %u x %u, res=%.3f, origin=(%.2f, %.2f, %.2f)",
             msg_->info.width, msg_->info.height, msg_->info.resolution,
             msg_->info.origin.position.x,
             msg_->info.origin.position.y,
             msg_->info.origin.position.z);
    
    map_ptr->loadOccupancyGrid(msg_);

    localizer.setMap(map_ptr);

  }
}

void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_) {
  /**
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   * You can check ros_bridge.h for helps :)
   */

  // TODO


    Eigen::Isometry2f iso;
    pose2isometry(msg_->pose.pose,iso);
    localizer.setInitialPose(iso);
    }


void callback_scan(const sensor_msgs::LaserScanConstPtr& msg_) {
  /**
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */
  // TODO

  Localizer2D::ContainerType scanned_points;

  scan2eigen(msg_, scanned_points);


  
  // * Set the laser parameters and process the incoming scan through the
  // * localizer

  //Set laser parameters in localizer

  //parameters
  float r_min = msg_ -> range_min;
  float r_max = msg_ -> range_max;
  float a_min = msg_ -> angle_min;
  float a_max = msg_ -> angle_max;
  float a_incr = msg_ -> angle_increment;

  localizer.setLaserParams(r_min, r_max, a_min, a_max, a_incr);

  //Process incoming scan 
  localizer.process(scanned_points);
   
  // TODO

  /**
   * Send a transform message between FRAME_WORLD and FRAME_LASER.
   * The transform should contain the pose of the laser with respect to the map
   * frame.
   * You can use the isometry2transformStamped function to convert the isometry
   * to the right message type.
   * Look at include/constants.h for frame names
   *
   * The timestamp of the message should be equal to the timestamp of the
   * received message (msg_->header.stamp)
   */

  Eigen::Isometry2f curr_estimate = localizer.X();
  
  //Initialize transformStamped message
  geometry_msgs::TransformStamped transform_stamped_message;

  isometry2transformStamped(curr_estimate, transform_stamped_message, FRAME_WORLD, FRAME_LASER, msg_->header.stamp);


  static tf2_ros::TransformBroadcaster br;

  // broadcast the transform
  br.sendTransform(transform_stamped_message);
  // TODO

  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
  // TODO

  nav_msgs::Odometry odometry_message;

  transformStamped2odometry(transform_stamped_message, odometry_message);

  pub_odom.publish(odometry_message);


  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);
}