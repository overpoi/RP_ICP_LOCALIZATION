#pragma once
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Dense>
#include <vector>

void scan2eigen(const sensor_msgs::LaserScan::ConstPtr& msg_,
                std::vector<Eigen::Vector2f,
                            Eigen::aligned_allocator<Eigen::Vector2f>>& dest_);  //sensor_msg::LaserScan (polar) --> 2d Eigen Vector (ICP input)

void eigen2scan(
    const std::vector<Eigen::Vector2f,
                      Eigen::aligned_allocator<Eigen::Vector2f>>& src_,
    sensor_msgs::LaserScan& dest_, float range_min_, float range_max_,
    float angle_min_, float angle_max_, float angle_increment_); //Eigen vector --> Laser scan 

void isometry2transformStamped(const Eigen::Isometry2f& pose_, //translation + rotation
                               geometry_msgs::TransformStamped& msg_, //broadcast output
                               const std::string& frame_id_, //map
                               const std::string& child_frame_id_, //base_laser_link
                               const ros::Time& stamp_); //scan time 

void transformStamped2odometry(const geometry_msgs::TransformStamped& msg_,
                               nav_msgs::Odometry& odom_); //transform broadcast --> odometry 

void pose2isometry(const geometry_msgs::Pose& pose_, Eigen::Isometry2f& iso_); //(iniital) pose --> Eigen 2d