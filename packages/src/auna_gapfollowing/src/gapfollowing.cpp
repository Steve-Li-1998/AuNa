#include "auna_gapfollowing/gapfollowing.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iterator>
#include <optional>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

GapFollow::GapFollow() : rclcpp::Node("gap_following", rclcpp::NodeOptions().use_clock_thread(true))
{
  this->declare_parameters();

  this->period_ = std::chrono::milliseconds(static_cast<int>(1000.0 / vel_pub_rate_));

  this->scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_, 1, std::bind(&GapFollow::scan_callback, this, std::placeholders::_1));
  this->vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(this->vel_topic_, 10);
  this->timer_ = this->create_wall_timer(period_, std::bind(&GapFollow::timer_callback, this));
}

void GapFollow::declare_parameters()
{
  // Declare topic names
  this->declare_parameter("scan_topic", std::string("scan"));
  this->declare_parameter("vel_topic", std::string("cmd_vel/gap"));

  // Get topic names
  this->scan_topic_ = this->get_parameter("scan_topic").as_string();
  this->vel_topic_ = this->get_parameter("vel_topic").as_string();

  // Declare parameter names
  this->declare_parameter("vel_pub_rate", 50);

  // Get parameter names
  this->vel_pub_rate_ = this->get_parameter("vel_pub_rate").as_double();
}

void GapFollow::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  this->scan_msg_ = scan_msg;
}

void GapFollow::timer_callback()
{
  rclcpp::Time t_msg(this->scan_msg_->header.stamp);  // message release time
  rclcpp::Time t_now = this->get_clock()->now();      // current time
  auto delay = t_now - t_msg;
  if (delay > rclcpp::Duration(std::chrono::milliseconds(500))) {
    this->stop_robot();
    RCLCPP_ERROR(this->get_logger(), "The scan data is too old, robot will stop.");
  } else if (this->last_scan_time_ == t_msg) {
    return;
  } else {
    this->last_scan_time_ = t_msg;
    auto & ranges = this->scan_msg_->ranges;
    if (this->scan_msg_) {
      this->preprocess_scan(*(this->scan_msg_));
      auto target_gap = this->find_target_gap(this->find_gap(*(this->scan_msg_)));
      if (target_gap.has_value()) {
        auto [linear_x, angular_z] = this->compute_velocity(target_gap.value(), *(this->scan_msg_));

      } else {
        this->stop_robot();
        RCLCPP_ERROR(this->get_logger(), "No avaliable destination.");
      }
    } else {
      this->stop_robot();
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "There is still no message from topic \"" << this->scan_topic_ << "\" cached.");
    }
  }
}

void GapFollow::preprocess_scan(sensor_msgs::msg::LaserScan & msg)
{
  auto & ranges = msg.ranges;
  const unsigned int bubble_width = static_cast<unsigned int>(ranges.size() * 0.03);
  auto it = std::min_element(ranges.begin(), ranges.end());
  long long min_index = std::distance(ranges.begin(), it);
  for (int i = min_index - bubble_width; i <= static_cast<int>(min_index + bubble_width); ++i) {
    ranges[i] = 0.0;
  }
}

std::vector<std::pair<long long, long long>> GapFollow::find_gap(
  const sensor_msgs::msg::LaserScan & msg) const
{
  const auto & ranges = msg.ranges;
  std::vector<std::pair<long long, long long>> gaps;
  gaps.reserve(10);
  auto in_gap = false;
  for (size_t i = 0; i < ranges.size(); ++i) {
    if (in_gap && ranges[i] == 0.0) {
      gaps.back().second = static_cast<long long>(i - 1);
      in_gap = false;
    }
    if (!in_gap && ranges[i] != 0.0) {
      gaps.emplace_back(static_cast<long long>(i), 0);
      in_gap = true;
    }
  }

  // Case the last gap not closed
  if (in_gap) {
    gaps.back().second = static_cast<long long>(ranges.size()) - 1;
  }
  return gaps;
}

std::optional<std::pair<long long, long long>> GapFollow::find_target_gap(
  const std::vector<std::pair<long long, long long>> & gaps) const
{
  if (gaps.empty()) {
    return std::nullopt;
  }

  // gaps.erase(
  //   std::remove_if(
  //     gaps.begin(), gaps.end(),
  //     [&msg](const auto & item) {
  //       return (
  //         item.second * msg.angle_increment + msg.angle_min < -M_PI / 2.0 ||
  //         item.first * msg.angle_increment + msg.angle_min > M_PI / 2.0);
  //     }),
  //   gaps.end());

  auto target = std::max_element(gaps.begin(), gaps.end(), [](const auto & a, const auto & b) {
    return (a.second - a.first) < (b.second - b.first);
  });
  return *target;
}

void GapFollow::stop_robot() const
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0;
  msg.linear.z = 0;
  this->vel_pub_->publish(msg);
}

std::pair<double, double> GapFollow::compute_velocity(
  const std::pair<long long, long long> & target_gap, const sensor_msgs::msg::LaserScan & msg) const
{
  const auto & ranges = msg.ranges;
  std::span<const float> gap_content(
    ranges.begin() + target_gap.first, ranges.begin() + target_gap.second + 1);
  auto max_range = std::max_element(gap_content.begin(), gap_content.end());
  // TODO compute the velocity command by give point
  return {2.0, 1.0};
}