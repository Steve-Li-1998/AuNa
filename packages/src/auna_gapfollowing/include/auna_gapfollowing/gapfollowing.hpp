#ifndef AUNA_GAPFOLLOWING__GAPFOLLOWING_HPP_
#define AUNA_GAPFOLLOWING__GAPFOLLOWING_HPP_

#include "rclcpp/rclcpp.hpp"

#include <rclcpp/experimental/buffers/ring_buffer_implementation.hpp>
#include <rclcpp/parameter.hpp>
// #include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
// #include "nav_msgs/msg/odometry.hpp"
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
  // /**
  //  * @brief A lightweight view providing circular access to a std::vector.
  //  *
  //  * This class allows accessing elements of a vector as if it were a ring buffer
  //  * (i.e., indices wrap around automatically using modulo arithmetic).
  //  * It does not own the underlying data; it only provides a view.
  //  *
  //  * @tparam T The element type stored in the underlying vector.
  //  */
  // template <typename T>
  // class RingBufferView
  // {
  // public:
  //   /**
  //    * @brief Constructs a ring buffer view over an existing vector.
  //    * @param data Reference to the vector providing storage.
  //    */
  //   RingBufferView(std::vector<T> & data) : data_(data) {}

  //   /// Default copy constructor.
  //   RingBufferView(const RingBufferView & other) = default;

  //   /**
  //    * @brief Returns the number of elements in the buffer.
  //    * @return Size of the underlying vector.
  //    */
  //   std::size_t size() const { return data_.size(); }

  //   /**
  //    * @brief Provides non-const access to an element by circular index.
  //    * @param i Index (will be wrapped using modulo `size()`).
  //    * @return Reference to the element at wrapped index.
  //    */
  //   T & operator[](size_t i) { return data_[i % data_.size()]; }

  //   /**
  //    * @brief Provides const access to an element by circular index.
  //    * @param i Index (will be wrapped using modulo `size()`).
  //    * @return Const reference to the element at wrapped index.
  //    */
  //   const T & operator[](size_t i) const { return data_[i % data_.size()]; }

  //   /**
  //    * @brief Returns a reference to the underlying vector.
  //    * @return Reference to the backing std::vector.
  //    */
  //   std::vector<T> & data() { return data_; }

  //   /**
  //    * @brief Returns a const reference to the underlying vector.
  //    * @return Const reference to the backing std::vector.
  //    */
  //   const std::vector<T> & data() const { return data_; }

  // private:
  //   std::vector<T> & data_;
  // };

  // Controller parameters
  double vel_pub_rate_;
  std::chrono::milliseconds period_;
  constexpr static double SAFE_DISTANCE_ = 0.5;
  double bubble_radius_ratio_;
  float linear_velocity_factor_;
  float angular_velocity_factor_;
  float max_linear_velocity_;
  float max_angular_velocity_;
  // double desired_distance_;
  // double velocity_;
  // double max_steering_angle_;
  // double min_velocity_;
  // double max_velocity_;
  // double error_threshold_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
  rclcpp::Time last_scan_time_;

  // Topic names
  std::string scan_topic_;
  std::string vel_topic_;

  // ROS2 interfaces
  // rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Callback for laser scan messages
   * @param scan_msg Laser scan message
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

  void timer_callback();

  void preprocess_scan(sensor_msgs::msg::LaserScan & msg);

  /**
   * @brief Finds continuous non-zero intervals ("gaps") in a ring buffer of ranges.
   *
   * This function scans through a circular buffer of floating-point range values
   * and identifies all continuous segments where the values are non-zero.
   * Each segment is represented as a pair of indices `(start, end)` marking
   * the inclusive range of the gap.
   *
   * A gap:
   * - Starts when a transition from `0.0` to a non-zero value occurs.
   * - Ends when a transition from a non-zero value back to `0.0` occurs.
   *
   * @param ranges A ring buffer view of floating-point values.
   *               - `0.0` indicates an empty measurement.
   *               - Non-zero indicates a valid measurement.
   *
   * @return A vector of pairs `(start_index, end_index)` representing detected gaps.
   *         If no gaps are found, the vector will be empty.
   */
  std::vector<std::pair<size_t, size_t>> find_gap(const sensor_msgs::msg::LaserScan & msg) const;

  /**
   * @brief Find the gap with the maximum length from a list of gaps.
   *
   * This function inspects a vector of intervals, where each interval is represented
   * as a pair of two size_t values (start, end). It computes the gap length as
   * `(second - first)` for each pair and returns the interval with the maximum length.
   *
   * @param gaps Reference to a vector of gap intervals (start, end).
   *
   * @return std::optional<std::pair<size_t, size_t>>
   *         - The interval with the maximum length if the input is not empty.
   *         - std::nullopt if the input vector is empty.
   */
  std::optional<std::pair<size_t, size_t>> find_target_gap(
    const std::vector<std::pair<size_t, size_t>> & gaps) const;

  void stop_robot() const;
  void send_vel_cmd(float linear_x, float angular_z) const;

  std::pair<float, float> compute_velocity(
    const std::pair<size_t, size_t> & target_gap, const sensor_msgs::msg::LaserScan & msg) const;

  std::pair<float, float> scaleToLimits(float & linear, float & angular) const;

  /**
   * @brief Declare and get parameters from parameter server
   */
  void declare_parameters();
};

#endif  // AUNA_GAPFOLLOWING__GAPFOLLOWING_HPP_
