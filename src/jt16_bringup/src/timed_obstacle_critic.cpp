#include "jt16_bringup/timed_obstacle_critic.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{
namespace
{
bool validMovingCloud(const sensor_msgs::msg::PointCloud2 & message)
{
  if (message.height != 1 || message.is_bigendian || message.point_step != 16 ||
    message.row_step != uint64_t(message.width) * 16 ||
    message.data.size() != message.row_step || message.fields.size() != 4)
  {
    return false;
  }
  const std::array<std::string, 4> names{"x", "y", "vx", "vy"};
  for (size_t i = 0; i < names.size(); ++i) {
    const auto & f = message.fields[i];
    if (f.name != names[i] || f.offset != 4 * i || f.count != 1 ||
      f.datatype != sensor_msgs::msg::PointField::FLOAT32) {return false;}
  }
  return message.header.stamp.sec >= 0;
}
}  // namespace

void TimedObstacleCritic::initialize()
{
  const auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(weight_, "cost_weight", 100.0f);
  getParam(clearance_, "clearance_m", 0.35f);
  if (!std::isfinite(weight_) || weight_ <= 0 ||
    !std::isfinite(clearance_) || clearance_ <= 0)
  {
    throw std::runtime_error("Invalid timed obstacle critic parameters");
  }
  const double inf = std::numeric_limits<double>::infinity();
  bounds_ = {inf, inf, -inf, -inf};
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.empty()) {throw std::runtime_error("Missing timed critic footprint");}
  for (const auto & p : footprint) {
    bounds_[0] = std::min(bounds_[0], p.x); bounds_[1] = std::min(bounds_[1], p.y);
    bounds_[2] = std::max(bounds_[2], p.x); bounds_[3] = std::max(bounds_[3], p.y);
  }
  subscription_ = parent_.lock()->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/moving_obstacle_points", rclcpp::QoS(1),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {receive(*m);});
}

void TimedObstacleCritic::receive(const sensor_msgs::msg::PointCloud2 & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  points_.clear(); valid_ = false;
  if (message.header.frame_id != costmap_ros_->getGlobalFrameID() || !validMovingCloud(message)) {
    return;
  }
  for (size_t offset = 0; offset < message.data.size(); offset += 16) {
    std::array<float, 4> point;
    std::memcpy(point.data(), message.data.data() + offset, 16);
    if (!std::all_of(point.begin(), point.end(), [](float v) {return std::isfinite(v);}) ||
      std::hypot(point[2], point[3]) > 1.2f)
    {
      points_.clear(); return;
    }
    points_.push_back(point);
  }
  stamp_ = rclcpp::Time(message.header.stamp, parent_.lock()->get_clock()->get_clock_type());
  valid_ = true;
}

double TimedObstacleCritic::poseRisk(double x, double y, double yaw, double horizon) const
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  double nearest_squared = double(clearance_) * clearance_;
  // shortcut: constant-velocity visible surfaces only; extend for occlusion or turning movers.
  for (const auto & p : points_) {
    const double dx = p[0] + p[2] * horizon - x, dy = p[1] + p[3] * horizon - y;
    const double bx = c * dx + s * dy, by = -s * dx + c * dy;
    const double gx = std::max({bounds_[0] - bx, 0.0, bx - bounds_[2]});
    const double gy = std::max({bounds_[1] - by, 0.0, by - bounds_[3]});
    nearest_squared = std::min(nearest_squared, gx * gx + gy * gy);
  }
  const double risk = std::max(0.0, 1.0 - std::sqrt(nearest_squared) / clearance_);
  return risk * risk;
}

void TimedObstacleCritic::score(CriticData & data)
{
  if (!enabled_) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  const auto node = parent_.lock();
  const double age = (node->now() - stamp_).seconds();
  const rclcpp::Time pose_stamp(data.state.pose.header.stamp, node->get_clock()->get_clock_type());
  const double pose_age = (node->now() - pose_stamp).seconds();
  const size_t steps = data.trajectories.x.shape(1);
  if (!valid_ || age < -0.03 || age > 0.5 || pose_age < -0.03 || pose_age > 0.5 ||
    data.state.pose.header.frame_id != costmap_ros_->getGlobalFrameID() ||
    !std::isfinite(data.model_dt) || data.model_dt <= 0 || steps == 0)
  {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "TimedObstacleCritic rejected input: valid=%d cloud_age=%.3f pose_age=%.3f frame=%s dt=%.3f steps=%zu",
      valid_, age, pose_age, data.state.pose.header.frame_id.c_str(), data.model_dt, steps);
    data.fail_flag = true; return;
  }
  const double offset = (pose_stamp - stamp_).seconds();
  for (size_t i = 0; i < data.costs.size(); ++i) {
    double peak = 0.0, total = 0.0;
    for (size_t j = 0; j < steps; ++j) {
      const double risk = poseRisk(data.trajectories.x(i, j), data.trajectories.y(i, j),
        data.trajectories.yaws(i, j), offset + (j + 1) * data.model_dt);
      peak = std::max(peak, risk); total += risk;
    }
    data.costs(i) += weight_ * (0.5 * peak + total / steps);
  }
}
}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::TimedObstacleCritic, mppi::critics::CriticFunction)
