#include "2d_mapping.h"

mapping_2d::mapping_2d() : Node("lidar_matching")
{
  // Get parameters
  this->declare_parameter("points_frame_id", "laser_link");
  this->declare_parameter("map_frame_id", "map");  // 添加父坐标系参数
  this->declare_parameter("min_add_scan_shift", 1.0);
  this->declare_parameter("min_add_scan_angle", 0.2);
  this->declare_parameter("map_resolution", 0.03);
  this->declare_parameter("real_width", 9);
  this->declare_parameter("real_height", 12);
  this->declare_parameter("angle_resolution", 0.174533);
  this->declare_parameter("step_size", 6);
  
  points_frame_id = this->get_parameter("points_frame_id").as_string();
  map_frame_id = this->get_parameter("map_frame_id").as_string();
  min_add_scan_shift_ = this->get_parameter("min_add_scan_shift").as_double();
  min_add_scan_angle_ = this->get_parameter("min_add_scan_angle").as_double();
  map_resolution = this->get_parameter("map_resolution").as_double();
  real_width = this->get_parameter("real_width").as_int();
  real_height = this->get_parameter("real_height").as_int();
  angle_resolution = this->get_parameter("angle_resolution").as_double();
  step_size = this->get_parameter("step_size").as_int();
  map_width  = static_cast<int>(std::ceil(real_height  / map_resolution));
  map_height  = static_cast<int>(std::ceil(real_width / map_resolution));
  GridMapConverter = new RadarToGridMapConverter(map_resolution, map_width, map_height, step_size, angle_resolution);

  // Create subscribers and publishers
  points_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 100000, std::bind(&mapping_2d::points_callback, this, std::placeholders::_1));
  
  current_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/current_pose", 1000);
  grid_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map", 10);
  
  // Initialize TF2 broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  grid_map.assign(map_width * map_height, 0);
  current_pose_.x = current_pose_.y = current_pose_.z = 0.0;
  current_pose_.roll = current_pose_.pitch = current_pose_.yaw = 0.0;
  previous_pose_.x = previous_pose_.y = previous_pose_.z = 0.0;
  previous_pose_.roll = previous_pose_.pitch = previous_pose_.yaw = 0.0;

}

mapping_2d::~mapping_2d(){} 

void mapping_2d::points_callback(const sensor_msgs::msg::LaserScan::SharedPtr input)
{
  static int frame=0;
  timeLaserInfoStamp = input->header.stamp;
  if (cloudKeyPoses3D.empty())
  {
    probability(input);
    GridMapConverter->setInputTarget(input);
  }

  GridMapConverter->align(current_pose_); //预测的当前帧坐标
  GridMapConverter->setInputSource(input);

  current_pose_ = GridMapConverter->getpose();

  //创建变换
  auto transform = createTransform(current_pose_.x, current_pose_.y, current_pose_.z, current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  //发送变换
  tf_broadcaster_->sendTransform(transform);

  if(saveKey())
  {
    std::cout<<"frame: "<<frame++<<std::endl;
    probability(input);
    GridMapConverter->setInputTarget(input);
    pub_rviz();
  }
}

bool mapping_2d::saveKey()
{
  if (saveFrame() == false)
    return false;

  cloudKeyPoses3D.push_back(current_pose_);
  previous_pose_ = current_pose_;
  return true;
}

bool mapping_2d::saveFrame()
{
  if (cloudKeyPoses3D.empty())
    return true;

  Eigen::Affine3f transStart = pcl::getTransformation(previous_pose_.x, previous_pose_.y, previous_pose_.z, previous_pose_.roll, previous_pose_.pitch, previous_pose_.yaw);
  Eigen::Affine3f transFinal = pcl::getTransformation(current_pose_.x, current_pose_.y, current_pose_.z, current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
  //std::cout<<"shift: "<<sqrt(x*x + y*y + z*z)<<std::endl;
  if (abs(yaw) < min_add_scan_angle_ && sqrt(x*x + y*y + z*z) < min_add_scan_shift_)
  {
    return false;
  }
  return true;
}

geometry_msgs::msg::TransformStamped mapping_2d::createTransform(float x, float y, float z, float roll, float pitch, float yaw)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = map_frame_id;
  transform.child_frame_id = points_frame_id;
  
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  return transform;
}

void mapping_2d::pub_rviz()
{
  auto grid_map_msg = nav_msgs::msg::OccupancyGrid();
  grid_map_msg.header.stamp = this->now();
  grid_map_msg.header.frame_id = "map";
  grid_map_msg.info.resolution = map_resolution;
  grid_map_msg.info.width = map_width;
  grid_map_msg.info.height = map_height;
  grid_map_msg.info.origin.position.x = -map_width * map_resolution / 2;
  grid_map_msg.info.origin.position.y = -map_height * map_resolution / 2;
  grid_map_msg.data = grid_map;
  grid_map_pub_->publish(grid_map_msg);
}

void mapping_2d::probability(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    float range = msg->ranges[i];
    float angle = msg->angle_min + i * msg->angle_increment + current_pose_.yaw;

    if (std::isfinite(range))
    {
      int x = (range * cos(angle) + current_pose_.x) / map_resolution + map_width/2;
      int y = (range * sin(angle) + current_pose_.y) / map_resolution + map_height/2;
      if (x >= 0 && x < map_width && y >= 0 && y < map_height)
      {
        if(grid_map[x + y * map_width] != 100)
        {
          GridMapConverter->setGridValue(x, y, 100,grid_map); // Set the original laser point to 100
        }
      }
    }
  }
}
