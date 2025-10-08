#include "auna_gapfollowing/gapfollowing.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <rcl/time.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <iterator>
#include <optional>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

GapFollow::GapFollow()
: rclcpp::Node("gap_following", rclcpp::NodeOptions().use_clock_thread(true)), scan_msg_(nullptr)
{
  this->declare_parameters();
  std::cout << this->get_name() << std::endl;

  this->period_ = std::chrono::milliseconds(static_cast<int>(1000.0 / vel_pub_rate_));
  this->last_scan_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  this->scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_, 1, std::bind(&GapFollow::scan_callback, this, std::placeholders::_1));
  this->vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(this->vel_topic_, 10);
  this->timer_ = this->create_wall_timer(period_, std::bind(&GapFollow::timer_callback, this));
}

void GapFollow::declare_parameters()
{
  // Declare topic names
  this->declare_parameter("scan_topic", std::string("robot1/scan"));
  this->declare_parameter("vel_topic", std::string("robot1/cmd_vel/gap"));

  // Get topic names
  this->scan_topic_ = this->get_parameter("scan_topic").as_string();
  RCLCPP_INFO(this->get_logger(), "Subscribing to scan topic: %s", this->scan_topic_.c_str());
  this->vel_topic_ = this->get_parameter("vel_topic").as_string();
  RCLCPP_INFO(this->get_logger(), "Publishing velocity to topic: %s", this->vel_topic_.c_str());

  // Declare controller parameters
  this->declare_parameter("linear_velocity_factor", 1.0);
  this->declare_parameter("angular_velocity_factor", 1.0);
  this->declare_parameter("max_linear_velocity", 10.5);
  this->declare_parameter("max_angular_velocity", 10.0);
  this->declare_parameter("bubble_radius_ratio", 0.2);

  // Get controller parameters
  this->bubble_radius_ratio_ = this->get_parameter("bubble_radius_ratio").as_double();
  RCLCPP_INFO(this->get_logger(), "Bubble radius ratio set to: %.3f", this->bubble_radius_ratio_);
  this->linear_velocity_factor_ = this->get_parameter("linear_velocity_factor").as_double();
  RCLCPP_INFO(
    this->get_logger(), "Linear velocity factor set to: %.3f", this->linear_velocity_factor_);
  this->angular_velocity_factor_ = this->get_parameter("angular_velocity_factor").as_double();
  RCLCPP_INFO(
    this->get_logger(), "Angular velocity factor set to: %.3f", this->angular_velocity_factor_);
  this->max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
  RCLCPP_INFO(this->get_logger(), "Max linear velocity set to: %.3f", this->max_linear_velocity_);
  this->max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
  RCLCPP_INFO(this->get_logger(), "Max angular velocity set to: %.3f", this->max_angular_velocity_);

  // Declare velocity publish rate
  this->declare_parameter("vel_pub_rate", 50.0);

  // Get velocity publish rate
  this->vel_pub_rate_ = this->get_parameter("vel_pub_rate").as_double();
}

void GapFollow::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  this->scan_msg_ = scan_msg;
}

void GapFollow::timer_callback()
{
  if (!this->scan_msg_) {
    this->stop_robot();
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "There is still no message from topic \"" << this->scan_topic_ << "\" cached.");
    return;
  }
  rclcpp::Time t_msg(this->scan_msg_->header.stamp);  // message release time
  rclcpp::Time t_now = this->get_clock()->now();      // current time
  if (t_now.seconds() == 0.0) {
    RCLCPP_WARN(this->get_logger(), "ROS time not initialized yet");
    return;
  }
  auto delay = t_now - t_msg;

  if (delay > rclcpp::Duration(std::chrono::milliseconds(500))) {
    this->stop_robot();
    RCLCPP_ERROR(this->get_logger(), "The scan data is too old, robot will stop.");
  } else if (this->last_scan_time_ == t_msg) {
    return;
  } else {
    this->last_scan_time_ = t_msg;
    this->preprocess_scan(*(this->scan_msg_));
    auto target_gap = this->find_target_gap(this->find_gap(*(this->scan_msg_)));
    if (target_gap.has_value()) {
      auto [linear_x, angular_z] = this->compute_velocity(target_gap.value(), *(this->scan_msg_));
      this->send_vel_cmd(linear_x, angular_z);
    } else {
      this->stop_robot();
      RCLCPP_ERROR(this->get_logger(), "No avaliable destination.");
    }
  }
}

void GapFollow::preprocess_scan(sensor_msgs::msg::LaserScan & msg)
{
  // confim bubble width
  auto & ranges = msg.ranges;
  const auto ranges_copy = ranges;
  const size_t bubble_radius = static_cast<size_t>(ranges.size() * this->bubble_radius_ratio_);

  // Replace  inf values with max range
  std::replace_if(
    ranges.begin(), ranges.end(), [](float r) { return std::isinf(r); }, msg.range_max);

  // Zero out values within the "bubble" around the danger points
  for (size_t i = 0; i < ranges.size(); ++i) {
    const auto & range = ranges_copy[i];
    if (range < this->SAFE_DISTANCE_) {
      size_t start_index, end_index;
      if (bubble_radius > i) {
        start_index = 0;
      } else {
        start_index = i - bubble_radius;
      }
      end_index = std::min(i + bubble_radius + 1, ranges.size());

      for (size_t j = start_index; j < end_index; ++j) {
        ranges[j] = 0.0;
      }
    }
  }
}

std::vector<std::pair<size_t, size_t>> GapFollow::find_gap(
  const sensor_msgs::msg::LaserScan & msg) const
{
  const auto & ranges = msg.ranges;
  std::vector<std::pair<size_t, size_t>> gaps;
  gaps.reserve(10);
  auto in_gap = false;
  for (size_t i = 0; i < ranges.size(); ++i) {
    if (in_gap && ranges[i] == 0.0) {
      gaps.back().second = static_cast<size_t>(i - 1);
      in_gap = false;
    }
    if (!in_gap && ranges[i] != 0.0) {
      gaps.emplace_back(static_cast<size_t>(i), 0);
      in_gap = true;
    }
  }

  // Case the last gap not closed
  if (in_gap) {
    gaps.back().second = static_cast<size_t>(ranges.size()) - 1;
  }
  return gaps;
}

std::optional<std::pair<size_t, size_t>> GapFollow::find_target_gap(
  const std::vector<std::pair<size_t, size_t>> & gaps) const
{
  if (gaps.empty()) {
    return std::nullopt;
  }

  auto target = std::max_element(gaps.begin(), gaps.end(), [](const auto & a, const auto & b) {
    return (a.second - a.first) < (b.second - b.first);
  });
  return *target;
}

void GapFollow::stop_robot() const { this->send_vel_cmd(0.0, 0.0); }

void GapFollow::send_vel_cmd(double linear_x, double angular_z) const
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  this->vel_pub_->publish(msg);
}

std::pair<double, double> GapFollow::compute_velocity(
  const std::pair<size_t, size_t> & target_gap, const sensor_msgs::msg::LaserScan & msg) const
{
  // find the furthest point in the scan
  const auto & ranges = msg.ranges;
  std::span<const float> gap_content(
    ranges.begin() + target_gap.first, ranges.begin() + target_gap.second + 1);
  auto max_range_it = std::max_element(gap_content.begin(), gap_content.end());

  // compute the angle and distance to the furthest point
  auto max_range_index = std::distance(ranges.data(), &*max_range_it);
  auto destination_angle = msg.angle_min + msg.angle_increment * max_range_index;
  auto destination_linear = msg.ranges[max_range_index];

  // scale the velocity by factors
  return this->scaleToLimits(
    this->linear_velocity_factor_ * destination_linear,
    this->angular_velocity_factor_ * destination_angle);
}

std::pair<double, double> GapFollow::scaleToLimits(double linear, double angular) const
{
  // Check for NaN values in the input velocities
  if (std::isnan(linear) || std::isnan(angular)) {
    RCLCPP_ERROR(
      this->get_logger(), "NaN detected in velocity! linear=%.3f angular=%.3f", linear, angular);
    return {0.0, 0.0};
  }

  // Compute the ratios of the absolute velocities to their respective limits
  double ratio_linear = std::abs(linear) / this->max_linear_velocity_;
  double ratio_angular = std::abs(angular) / this->max_angular_velocity_;
  double scale = std::max(ratio_linear, ratio_angular);

  // If either velocity exceeds its limit, scale both down proportionally
  if (scale > 1.0) {
    linear /= scale;
    angular /= scale;
  }

  return {linear, angular};
}