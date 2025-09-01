#include "localizer2d.h"
#include "icp/eigen_icp_2d.h"
#include <ros/ros.h> 
using namespace std;

Localizer2D::Localizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
  /**
   * If the map is initialized, fill the _obst_vect vector with world
   * coordinates of all cells representing obstacles.
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
  // TODO

  if (!_map){
    std::cerr << "No map initialized" << std::endl;
    return;
  }

  //data
  int rows = _map->rows();
  int cols = _map->cols();
  int resolution = _map->resolution();

  //Scan the map for occupied cells
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if ((*_map)(r,c) == CellType::Occupied) { 
        cv::Point2i grid_point(r,c); //create obstacle grid coord of obstacle
        std::cerr << "grid_point (" << grid_point.x << ", " << grid_point.y << ")" << std::endl;

        Eigen::Vector2f world_point = _map -> grid2world(grid_point); //map grid coord --> world coord
        std::cerr << "world_point (" << world_point[0] << ", " << world_point[1] << ")" << std::endl;

        _obst_vect.push_back(world_point); //obstacle coordinates vector
        
      }
    }
  }

  // Create KD-Tree
  // TODO
  _obst_tree_ptr = make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end()); //KD tree over obstacle points. from eigen_kdtree --> iterator, max poits in leaf default = 20 

  if(!_obst_tree_ptr) {
    ROS_INFO("Obst_tree_pointer is null!");
  }
} //setMap




/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  // TODO
  _laser_in_world = initial_pose_;   
}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  // Use initial pose to get a synthetic scan to compare with scan_
  // TODO
  ContainerType prediction;
  getPrediction(prediction); //predicted scan



  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  // TODO
  //ICP

  const int max_points_in_leaf= 20; //less leaves --> more precise
  ICP icp(prediction, scan_, max_points_in_leaf); //ICP from prediction to scan_, KD tree parameter
  icp.X() = _laser_in_world; //ICP point estimae = initial guess
  icp.run(50); 


  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
  // TODO

  _laser_in_world = icp.X(); //set after convergence
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();

  // Guard: KD-tree must be ready
  if (!_obst_tree_ptr) {
    ROS_WARN_THROTTLE(5.0, "KD-tree not ready; prediction skipped");
    return;
  }

  // Collect near map points from the KD-tree
  std::vector<PointType*> neighbours;
  const float search_radius = 10;  //10m
  _obst_tree_ptr->fullSearch(neighbours, _laser_in_world.translation(), search_radius);

  ROS_INFO_THROTTLE(5, "Prediction: found %zu nearby points", neighbours.size());

  // Check which points fall inside the sensor limits
  const Eigen::Isometry2f world_in_laser = _laser_in_world.inverse();

  for (PointType* p : neighbours) {
    const Eigen::Vector2f point_world_frame = *p; 
    const Eigen::Vector2f point_laser_frame = (world_in_laser * point_world_frame.homogeneous()).head<2>(); // for points in world frame transform in laser frame to check laser range and bearing

    const float dist = point_laser_frame.norm(); //range
    const float ang  = std::atan2(point_laser_frame.y(), point_laser_frame.x()); //bearing

    if (dist >= _range_min && dist <= _range_max &&
        ang  >= _angle_min && ang  <= _angle_max) { //check laser fov


      // Prediction in world frame
      prediction_.push_back(point_world_frame); //append to prediction the coord of visible obstacles in world frame
    }
  }
  
}

