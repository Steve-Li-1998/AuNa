#include "auna_laptimer/laptimer.hpp"

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

LapTimer::LapTimer() : rclcpp::Node("lap_timer"), last_ground_truth_position_(std::nullopt)
{
  this->declare_parameters();

  this->ground_truth_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->ground_truth_topic_, 10,
    std::bind(&LapTimer::ground_truth_callback, this, std::placeholders::_1));

  this->get_lap_records_srv_ = this->create_service<auna_laptimer_interfaces::srv::GetLapRecords>(
    this->get_lap_records_service_name_,
    std::bind(
      &LapTimer::get_lap_records_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void LapTimer::declare_parameters()
{
  // Declare topic names
  this->declare_parameter("ground_truth_topic", std::string("robot1/gazebo_pose"));
  this->declare_parameter("get_lap_records_service_name", std::string("get_lap_records"));

  // Get topic names
  this->ground_truth_topic_ = this->get_parameter("ground_truth_topic").as_string();
  this->get_lap_records_service_name_ =
    this->get_parameter("get_lap_records_service_name").as_string();

  // Declare start/end points of finish tapes (x1,y1,x2,y2)
  this->declare_parameter("finish_tapes", std::vector<double>{0.0, 0.0, 0.0, 1.0});

  // Get start/end points of finish tapes
  auto finish_tapes = this->get_parameter("finish_tapes").as_double_array();
  if (finish_tapes.size() % 4 != 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Finish tapes parameter must be a multiple of 6 (x1,y1,x2,y2)");
  } else {
    for (size_t i = 0; i < finish_tapes.size(); i += 4) {
      Eigen::Vector2d point1(finish_tapes[i], finish_tapes[i + 1]);
      Eigen::Vector2d point2(finish_tapes[i + 2], finish_tapes[i + 3]);
      this->finish_tapes_.emplace_back(point1, point2);
    }
    // Debug print
    RCLCPP_INFO(this->get_logger(), "Loaded %zu finish tapes", this->finish_tapes_.size());
    for (const auto & tape : this->finish_tapes_) {
      RCLCPP_INFO(
        this->get_logger(), "Tape from (%.2f, %.2f) to (%.2f, %.2f)", tape.first.x(),
        tape.first.y(), tape.second.x(), tape.second.y());
    }
  }
}

bool LapTimer::is_intersectant(const Line2d & path, const Line2d & final_tape)
{
  const auto & [a, b] = path;
  const auto & [c, d] = final_tape;
  Eigen::Vector3d vec_a(a.x(), a.y(), 0.0);
  Eigen::Vector3d vec_b(b.x(), b.y(), 0.0);
  Eigen::Vector3d vec_c(c.x(), c.y(), 0.0);
  Eigen::Vector3d vec_d(d.x(), d.y(), 0.0);
  Eigen::Vector3d ab = vec_b - vec_a;
  Eigen::Vector3d cd = vec_d - vec_c;
  Eigen::Vector3d ac = vec_c - vec_a;
  Eigen::Vector3d ad = vec_d - vec_a;
  Eigen::Vector3d ca = vec_a - vec_c;
  Eigen::Vector3d cb = vec_b - vec_c;
  Eigen::Vector3d norm_ab_1 = ab.cross(ac);
  Eigen::Vector3d norm_ab_2 = ab.cross(ad);
  Eigen::Vector3d norm_cd_1 = cd.cross(ca);
  Eigen::Vector3d norm_cd_2 = cd.cross(cb);
  if (
    (norm_ab_1.z() * norm_ab_2.z() <= 0.0) && (norm_cd_1.z() * norm_cd_2.z() <= 0.0) &&
    norm_cd_1.z() != 0.0) {
    return true;
  } else {
    return false;
  }
}

void LapTimer::get_lap_records_callback(
  const std::shared_ptr<auna_laptimer_interfaces::srv::GetLapRecords::Request> request,
  std::shared_ptr<auna_laptimer_interfaces::srv::GetLapRecords::Response> response)
{
  (void)request;
  auto & lap_records = response->lap_records.lap_records;
  lap_records.resize(this->lap_records_.size());
  for (size_t i = 0; i < this->lap_records_.size(); ++i) {
    const auto & [tape, time] = this->lap_records_[i];
    lap_records[i].tape.start.x = tape.first.x();
    lap_records[i].tape.start.y = tape.first.y();
    lap_records[i].tape.end.x = tape.second.x();
    lap_records[i].tape.end.y = tape.second.y();
    lap_records[i].stamp = time;
  }
  return;
}

void LapTimer::ground_truth_callback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr ground_truth_msg)
{
  // First message, just store the position
  if (this->last_ground_truth_position_ == std::nullopt) {
    this->last_ground_truth_position_.emplace(
      ground_truth_msg->pose.position.x, ground_truth_msg->pose.position.y);
    return;
  }
  // Subsequent messages, create a line segment from last to current position
  Eigen::Vector2d current_position(
    ground_truth_msg->pose.position.x, ground_truth_msg->pose.position.y);
  const Eigen::Vector2d & last_position = this->last_ground_truth_position_.value();

  Line2d robot_path = std::make_pair(last_position, current_position);

  // Check intersection with each finish tape
  for (const auto & tape : this->finish_tapes_) {
    if (this->is_intersectant(robot_path, tape)) {
      rclcpp::Time current_time(ground_truth_msg->header.stamp);
      this->lap_records_.emplace_back(tape, current_time);

      const auto sec = current_time.nanoseconds() / 1'000'000'000;
      const auto ms = (current_time.nanoseconds() % 1'000'000'000) / 1'000'000;

      RCLCPP_INFO_STREAM(
        this->get_logger(), "Car arrvied the tape (" << tape.first.x() << ", " << tape.first.y()
                                                     << ") - (" << tape.second.x() << ", "
                                                     << tape.second.y() << ") in [" << sec << " s "
                                                     << ms << " ms].");

      break;
    }
  }
  this->last_ground_truth_position_ = current_position;
}
