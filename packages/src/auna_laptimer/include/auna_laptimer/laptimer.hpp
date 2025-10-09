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

  /**
   * @brief Service callback to return all stored lap records.
   *
   * Called when the `GetLapRecords` service is requested. Copies every
   * recorded finish-line crossing (line segment + timestamp) from the
   * internal `lap_records_` vector into the service response.
   *
   * @param[in] request  Unused service request pointer.
   * @param[out] response Filled with all current lap records.
   *
   * @note Thread-safety is not guaranteed if accessed concurrently.
   *       Use a mutex if the callback runs in a multi-threaded context.
   */
  void get_lap_records_callback(
    const std::shared_ptr<auna_laptimer_interfaces::srv::GetLapRecords::Request> request,
    std::shared_ptr<auna_laptimer_interfaces::srv::GetLapRecords::Response> response);

  /**
   * @brief Callback for receiving ground-truth pose updates.
   *
   * Called whenever a new `PoseStamped` message is received. The function
   * checks if the robot’s path between the previous and current positions
   * intersects any defined finish tape. If a crossing is detected, a new
   * lap record (finish line + timestamp) is stored.
   *
   * @param[in] ground_truth_msg  The latest ground-truth pose message.
   *
   * @note Updates `last_ground_truth_position_` each call. The first message
   *       only initializes this position and does not check for crossings.
   */

  void ground_truth_callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr ground_truth_msg);

  /**
   * @brief Checks whether two 2D line segments intersect.
   *
   * Determines if the robot’s path segment and a finish tape segment
   * cross each other in the 2D plane using cross-product sign tests.
   *
   * @param[in] path        The line segment representing the robot's movement.
   * @param[in] final_tape  The line segment representing a finish tape.
   * @return true  If the two segments intersect.
   * @return false Otherwise.
   *
   * @note This method assumes both line segments lie on the XY plane (z = 0).
   */
  bool is_intersectant(const Line2d & line1, const Line2d & line2);

  /**
   * @brief Declare and get parameters from parameter server
   */
  void declare_parameters();
};

#endif  // AUNA_LAPTIMER__LAPTIMER_HPP_
