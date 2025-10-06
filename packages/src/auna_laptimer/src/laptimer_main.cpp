#include "auna_laptimer/laptimer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LapTimer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}