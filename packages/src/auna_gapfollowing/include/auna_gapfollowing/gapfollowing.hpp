#ifndef AUNA_GAPFOLLOWING__GAPFOLLOWING_HPP_
#define AUNA_GAPFOLLOWING__GAPFOLLOWING_HPP_

#include "rclcpp/rclcpp.hpp"

#include <rclcpp/experimental/buffers/ring_buffer_implementation.hpp>
#include <rclcpp/parameter.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class GapFollow : public rclcpp::Node
{
public:
  GapFollow();

private:
  // Controller parameters
  double vel_pub_rate_;
  std::chrono::milliseconds period_;
  constexpr static double SAFE_DISTANCE_ = 0.5;
  double bubble_radius_ratio_;
  float linear_velocity_factor_;
  float angular_velocity_factor_;
  float max_linear_velocity_;
  float max_angular_velocity_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
  rclcpp::Time last_scan_time_;

  // Topic names
  std::string scan_topic_;
  std::string vel_topic_;

  // ROS2 interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Callback for laser scan messages
   * @param scan_msg Laser scan message
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

  /**
   * @brief Timer callback to publish velocity commands at a fixed rate
   */
  void timer_callback();

  /**
   * @brief Preprocess the LaserScan data to remove unsafe and invalid readings.
   *
   * This function performs two key preprocessing steps:
   *  1. Replaces all infinite range readings (inf) with the sensor's maximum range value.
   *  2. Creates a "safety bubble" around detected obstacles by setting nearby range values
   *     to zero. This prevents the robot from attempting to navigate too close to obstacles.
   *
   * The bubble width is determined by `bubble_radius_ratio_`, which specifies the proportion
   * of scan points to include around each obstacle.
   *
   * @param msg LaserScan message containing the range data to preprocess. The ranges will be
   * modified in place.
   */

  void preprocess_scan(sensor_msgs::msg::LaserScan & msg);

  /**
   * @brief Identify continuous gaps from LaserScan data.
   *
   * This function scans through the laser range readings and identifies
   * contiguous sequences of valid (non-zero) measurements. Each continuous
   * segment is considered a navigable gap, defined by its start and end indices.
   * If a gap continues until the end of the scan, it is properly closed.
   *
   * @param msg LaserScan message containing distance readings.
   *
   * @return std::vector<std::pair<size_t, size_t>>
   *         - A list of gaps, where each pair represents the start and end indices.
   */
  std::vector<std::pair<size_t, size_t>> find_gap(const sensor_msgs::msg::LaserScan & msg) const;

  /**
   * @brief Find the widest gap from a list of detected gaps.
   *
   * This function iterates through a list of gaps, each represented by
   * a pair of start and end indices, and selects the one with the largest
   * width (difference between end and start). If no gaps are available,
   * an empty optional is returned.
   *
   * @param gaps A vector of pairs, where each pair represents the start
   *             and end indices of a detected gap.
   *
   * @return std::optional<std::pair<size_t, size_t>>
   *         - The start and end indices of the widest gap.
   *         - Returns std::nullopt if the list is empty.
   */
  std::optional<std::pair<size_t, size_t>> find_target_gap(
    const std::vector<std::pair<size_t, size_t>> & gaps) const;

  /**
   * @brief Stop the robot by sending zero velocity commands.
   *
   * This function halts the robot by publishing a Twist message
   * with both linear and angular velocities set to zero.
   */
  void stop_robot() const;

  /**
   * @brief Publish a velocity command to the robot.
   *
   * This function creates and publishes a geometry_msgs::msg::Twist message
   * to control the robot's linear and angular velocities.
   *
   * @param linear_x  Linear velocity along the x-axis (m/s).
   * @param angular_z Angular velocity around the z-axis (rad/s).
   */
  void send_vel_cmd(float linear_x, float angular_z) const;

  /**
   * @brief Compute the desired linear and angular velocity based on the target gap.
   *
   * This function identifies the furthest point within the selected gap region of the
   * LaserScan data and computes corresponding linear and angular velocities for the robot
   * to navigate toward that point. The linear velocity is scaled by distance and reduced
   * for sharp turns using a Gaussian damping function.
   *
   * @param target_gap A pair of indices representing the start and end of the selected gap.
   * @param msg The LaserScan message containing range and angular information.
   *
   * @return std::pair<float, float>
   *         - The first value is the computed linear velocity (m/s).
   *         - The second value is the computed angular velocity (rad/s).
   */
  std::pair<float, float> compute_velocity(
    const std::pair<size_t, size_t> & target_gap, const sensor_msgs::msg::LaserScan & msg) const;

  /**
   * @brief Clamp linear and angular velocities to safe operational limits.
   *
   * This function ensures that both linear and angular velocity commands
   * are within the predefined maximum bounds. It also checks for invalid
   * (NaN) input values and logs an error if detected, returning zero velocities
   * in that case to prevent unsafe behavior.
   *
   * @param linear  Reference to the desired linear velocity (m/s).
   * @param angular Reference to the desired angular velocity (rad/s).
   *
   * @return std::pair<float, float>
   *         - The clamped (linear, angular) velocity pair.
   *         - Returns {0.0, 0.0} if NaN values are detected.
   */
  std::pair<float, float> scaleToLimits(float & linear, float & angular) const;

  /**
   * @brief Declare and get parameters from parameter server
   */
  void declare_parameters();
};

#endif  // AUNA_GAPFOLLOWING__GAPFOLLOWING_HPP_
