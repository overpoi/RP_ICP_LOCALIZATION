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

using namespace std;

// Map, InitialPose, Scan 
//Void function callback pre definition
void callback_map(const nav_msgs::OccupancyGridConstPtr&);
void callback_initialpose(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
void callback_scan(const sensor_msgs::LaserScanConstPtr&);

shared_ptr<Map> map_ptr = nullptr;
ros::Publisher pub_scan, pub_odom;

// Map, InitialPose, Scan sub
ros::Subscriber map_sub, initial_pos_sub, base_scan_sub;

Localizer2D localizer;

int main(int argc, char** argv) {
  // TODO
  // Initialize ROS system
  ros::init(argc, argv, "localizer_node");

  // Create a NodeHandle to manage the node.
  // The namespace of the node is set to globalmap --> "/" 
  ros::NodeHandle nh("/");

  // Create shared pointer for the Map object
  // TODO
  map_ptr = std::make_shared<Map>();

  // * Subscribe to the topics:
  // * /map [nav_msgs::OccupancyGrid]
  // * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
  // * /base_scan [sensor_msgs::LaserScan]
  // * and assign the correct callbacks
  // *

  map_sub = nh.subscribe("/map", 10, callback_map);
  initial_pos_sub = nh.subscribe("/initialpose", 10, callback_initialpose);
  base_scan_sub = nh.subscribe("/base_scan", 10, callback_scan);
  
  // * Advertise the following topic:
  // * /odom_out [nav_msgs::Odometry]
  // TODO
  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_out", 10);
   
  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);

  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();

  return 0; //successful return 
}

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg_) {
  // If the internal map is not initialized, load the incoming occupancyGrid and
  // set the localizer map accordingly
  // Remember to load the map only once during the execution of the map.

  // TODO

  if(!map_ptr->initialized()){ //if map isnt initialized

    ROS_INFO("Map Initialized: %u x %u, res=%.3f, origin=(%.2f, %.2f, %.2f)",
             msg_->info.width, msg_->info.height, msg_->info.resolution,
             msg_->info.origin.position.x,
             msg_->info.origin.position.y,
             msg_->info.origin.position.z);
    
    map_ptr->loadOccupancyGrid(msg_); //load occupancy grid --> Unknown = -1, Free = 0, Occupied = 100

    localizer.setMap(map_ptr); //pass map to localizer

  }
}

void callback_initialpose(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_) { 
    Eigen::Isometry2f isometry_2d; //create empty isometry2d
    pose2isometry(msg_->pose.pose,isometry_2d); //convert pose 2 isometry
    localizer.setInitialPose(isometry_2d); //pass initial pose
  } 

  /**
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   * You can check ros_bridge.h for helps :)
   */

  // TODO

    //Convert PoseWihthCovarianceStamped msg --> Eigen isometry --> Inform localizer


void callback_scan(const sensor_msgs::LaserScanConstPtr& msg_) {
  /**
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */
  // TODO

  //Check that map is initialized before scan
  if (!map_ptr || !map_ptr->initialized()) {
    ROS_WARN_THROTTLE(10.0, "Map not initialized yet; skipping scan.");
    return;
  }

  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> scanned_points;
  scan2eigen(msg_, scanned_points); //Laserscan --> EigenPoints

  // * Set the laser parameters and process the incoming scan through the
  // * localizer

  //Set laser parameters in localizer
  //TODO
  float r_min = msg_ -> range_min; //min range value
  float r_max = msg_ -> range_max;  //max range value
  float a_min = msg_ -> angle_min; //starting angle of scan
  float a_max = msg_ -> angle_max; //end angle of scan
  float a_incr = msg_ -> angle_increment; //angular distance between measurements

  localizer.setLaserParams(r_min, r_max, a_min, a_max, a_incr);

  //Process incoming scan (scan points --> map points)
  localizer.process(scanned_points);
   
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

  const ros::Time stamp = msg_->header.stamp;


  Eigen::Isometry2f curr_estimate = localizer.X(); //get gurrent estimate
  
  //Create transformStamped message
  geometry_msgs::TransformStamped transform_stamped_message;

  //Convert Eigen transform --> TransformStamped message to broadcast
  isometry2transformStamped(curr_estimate, transform_stamped_message, FRAME_WORLD, FRAME_LASER, stamp);
  static tf2_ros::TransformBroadcaster br; //tf broadcaster
  br.sendTransform(transform_stamped_message);   // send broadcast the transform


  ROS_INFO_THROTTLE(20, "Broadcasted transform from %s frame to %s frame at time %.2f",
         transform_stamped_message.header.frame_id.c_str(),
         transform_stamped_message.child_frame_id.c_str(),
         transform_stamped_message.header.stamp.toSec());


  // TODO

  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
  // TODO

  //Transform stamped message to odometry message 
  nav_msgs::Odometry odometry_message; //create odometry msg 

  transformStamped2odometry(transform_stamped_message, odometry_message); //convert transformstamped to odometry

  pub_odom.publish(odometry_message);


  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  //Scan --> FRAME_LASER changing ID --> publish
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);
}