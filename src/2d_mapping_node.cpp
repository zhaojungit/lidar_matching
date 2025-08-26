#include "2d_mapping.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<mapping_2d>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}

