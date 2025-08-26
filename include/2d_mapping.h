#ifndef MAPPING
#define MAPPING

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <time.h>
#include <chrono>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// TF2 includes
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

// PCL includes
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

#include "laser2d.h"

class mapping_2d : public rclcpp::Node
{
public:
  mapping_2d();
  ~mapping_2d();

private:
  // ROS2 subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr points_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub_;
  
  // TF2 broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Timer
  rclcpp::Time timeLaserInfoStamp;
  std::vector<point> cloudKeyPoses3D;

  point current_pose_;
  point previous_pose_;
  std::string points_frame_id;
  std::string map_frame_id;
  float map_resolution;
  float angle_resolution;
  int real_width, real_height;
  int map_width, map_height;
  int step_size;
  float min_add_scan_shift_;
  float min_add_scan_angle_;
  std::vector<int8_t> grid_map;

  void probability(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void points_callback(const sensor_msgs::msg::LaserScan::SharedPtr input);
  geometry_msgs::msg::TransformStamped createTransform(float x, float y, float z, float roll, float pitch, float yaw);
  bool saveKey();
  bool saveFrame();
  void pub_rviz();
  RadarToGridMapConverter* GridMapConverter;


};

#endif // 2D_MAPPING

