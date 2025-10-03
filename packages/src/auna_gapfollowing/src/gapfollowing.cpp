#include "auna_gapfollowing/gapfollowing.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <rcl/time.h>

#include <algorithm>
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
  this->vel_topic_ = this->get_parameter("vel_topic").as_string();

  // Declare parameter names
  this->declare_parameter("vel_pub_rate", 50.0);

  // Get parameter names
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
  std::cout << t_msg.seconds() << std::endl;
  std::cout << t_msg.get_clock_type() << std::endl;
  std::cout << t_now.seconds() << std::endl;
  std::cout << t_now.get_clock_type() << std::endl;
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
  auto & ranges = msg.ranges;
  const unsigned int bubble_width = static_cast<unsigned int>(ranges.size() * 0.03);
  auto it = std::min_element(ranges.begin(), ranges.end());
  size_t min_index = std::distance(ranges.begin(), it);
  for (int i = min_index - bubble_width; i <= static_cast<int>(min_index + bubble_width); ++i) {
    ranges[i] = 0.0;
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

  // compute angle of destination
  auto max_range_index = std::distance(ranges.data(), &*max_range_it);
  auto destination_angle = msg.angle_min + msg.angle_increment * max_range_index;
  // get linear of destination
  auto destination_linear = msg.ranges[max_range_index];

  // TODO compute the velocity command by give point
  return {destination_linear, destination_angle};
}