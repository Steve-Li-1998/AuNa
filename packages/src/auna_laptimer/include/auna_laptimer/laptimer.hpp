#ifndef AUNA_LAPTIMER__LAPTIMER_HPP_
#define AUNA_LAPTIMER__LAPTIMER_HPP_

#include "auna_laptimer_interfaces/srv/get_lap_records.hpp"
#include "eigen3/Eigen/Dense"
#include "rclcpp/rclcpp.hpp"

#include <rclcpp/parameter.hpp>
#include <rclcpp/service.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class LapTimer : public rclcpp::Node
{
public:
  LapTimer();

private:
  using Line2d = std::pair<Eigen::Vector2d, Eigen::Vector2d>;

  std::vector<Line2d> finish_tapes_;

  std::vector<std::pair<Line2d, rclcpp::Time>> lap_records_;

  std::optional<Eigen::Vector2d> last_ground_truth_position_;

  // Topic names
  std::string ground_truth_topic_;

  // Services names
  std::string get_lap_records_service_name_;

  // ROS2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
  rclcpp::Service<auna_laptimer_interfaces::srv::GetLapRecords>::SharedPtr get_lap_records_srv_;

  void get_lap_records_callback(
    const std::shared_ptr<auna_laptimer_interfaces::srv::GetLapRecords::Request> request,
    std::shared_ptr<auna_laptimer_interfaces::srv::GetLapRecords::Response> response);

  void ground_truth_callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr ground_truth_msg);

  bool is_intersectant(const Line2d & line1, const Line2d & line2);

  /**
   * @brief Declare and get parameters from parameter server
   */
  void declare_parameters();
};

#endif  // AUNA_LAPTIMER__LAPTIMER_HPP_
