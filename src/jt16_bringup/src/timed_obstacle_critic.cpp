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
constexpr float kMaxObstacleSpeed = 1.2f;
constexpr double kObservationLifetime = .5;

bool validObstacleCloud(const sensor_msgs::msg::PointCloud2 & message)
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
  getParam(collision_cost_, "collision_cost", 1000000.0f);
  if (!std::isfinite(weight_) || weight_ <= 0 ||
    !std::isfinite(clearance_) || clearance_ <= 0 ||
    !std::isfinite(collision_cost_) || collision_cost_ <= 0)
  {
    throw std::runtime_error("Invalid timed obstacle critic parameters");
  }
  RCLCPP_INFO(logger_, "TimedObstacleCritic uses weight %.3f, clearance %.3f m",
    weight_, clearance_);
  const double inf = std::numeric_limits<double>::infinity();
  bounds_ = {inf, inf, -inf, -inf};
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.empty()) {throw std::runtime_error("Missing timed critic footprint");}
  for (const auto & p : footprint) {
    bounds_[0] = std::min(bounds_[0], p.x); bounds_[1] = std::min(bounds_[1], p.y);
    bounds_[2] = std::max(bounds_[2], p.x); bounds_[3] = std::max(bounds_[3], p.y);
  }
  subscription_ = parent_.lock()->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/forecast_surface_points", rclcpp::QoS(1),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {receive(*m);});
}

void TimedObstacleCritic::receive(const sensor_msgs::msg::PointCloud2 & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  points_.clear(); valid_ = false;
  if (message.header.frame_id != costmap_ros_->getGlobalFrameID() || !validObstacleCloud(message)) {
    return;
  }
  for (size_t offset = 0; offset < message.data.size(); offset += 16) {
    std::array<float, 4> point;
    std::memcpy(point.data(), message.data.data() + offset, 16);
    if (!std::all_of(point.begin(), point.end(), [](float v) {return std::isfinite(v);}) ||
      std::hypot(point[2], point[3]) > kMaxObstacleSpeed)
    {
      points_.clear(); return;
    }
    points_.push_back(point);
  }
  stamp_ = rclcpp::Time(message.header.stamp, parent_.lock()->get_clock()->get_clock_type());
  valid_ = true;
}

std::pair<double, bool> TimedObstacleCritic::poseRisk(
  double x, double y, double yaw, double horizon,
  const std::vector<std::array<float, 4>> & points) const
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  double nearest_squared = double(clearance_) * clearance_;
  // shortcut: constant-velocity visible surfaces only; extend for occlusion or turning movers.
  for (const auto & p : points) {
    const double dx = p[0] + p[2] * horizon - x, dy = p[1] + p[3] * horizon - y;
    const double bx = c * dx + s * dy, by = -s * dx + c * dy;
    const double gx = std::max({bounds_[0] - bx, 0.0, bx - bounds_[2]});
    const double gy = std::max({bounds_[1] - by, 0.0, by - bounds_[3]});
    double squared_gap = gx * gx + gy * gy;
    if (squared_gap == 0.) {return {1.0, true};}
    if (p[2] == 0.f && p[3] == 0.f) {
      // Zero marks unresolved motion; cap the near-term proximity cost, not future reachability.
      const double uncertainty = kMaxObstacleSpeed * std::clamp(horizon, 0., kObservationLifetime);
      const double gap = std::max(0., std::sqrt(squared_gap) - uncertainty);
      squared_gap = gap * gap;
    }
    nearest_squared = std::min(nearest_squared, squared_gap);
  }
  const double risk = std::max(0.0, 1.0 - std::sqrt(nearest_squared) / clearance_);
  return {risk * risk, false};
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
  if (!valid_ || age < -0.03 || age > kObservationLifetime || pose_age < -0.03 || pose_age > kObservationLifetime ||
    data.state.pose.header.frame_id != costmap_ros_->getGlobalFrameID() ||
    !std::isfinite(data.model_dt) || data.model_dt <= 0 || steps == 0 || data.costs.size() == 0)
  {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "TimedObstacleCritic rejected input: valid=%d cloud_age=%.3f pose_age=%.3f frame=%s dt=%.3f steps=%zu",
      valid_, age, pose_age, data.state.pose.header.frame_id.c_str(), data.model_dt, steps);
    data.fail_flag = true; return;
  }
  const double offset = (pose_stamp - stamp_).seconds();
  const auto [min_x, max_x] = std::minmax_element(data.trajectories.x.begin(), data.trajectories.x.end());
  const auto [min_y, max_y] = std::minmax_element(data.trajectories.y.begin(), data.trajectories.y.end());
  const double margin = std::hypot(std::max(std::abs(bounds_[0]), std::abs(bounds_[2])),
    std::max(std::abs(bounds_[1]), std::abs(bounds_[3]))) + clearance_ +
    kMaxObstacleSpeed * kObservationLifetime;
  std::vector<std::array<float, 4>> relevant;
  for (const auto & point : points_) {
    const double x0 = point[0] + point[2] * (offset + data.model_dt);
    const double x1 = point[0] + point[2] * (offset + steps * data.model_dt);
    const double y0 = point[1] + point[3] * (offset + data.model_dt);
    const double y1 = point[1] + point[3] * (offset + steps * data.model_dt);
    // A constant-velocity surface outside this batch's padded bounds cannot affect any pose.
    if (std::max(x0, x1) < *min_x - margin || std::min(x0, x1) > *max_x + margin ||
      std::max(y0, y1) < *min_y - margin || std::min(y0, y1) > *max_y + margin) {continue;}
    relevant.push_back(point);
  }
  size_t colliding = 0;
  for (size_t i = 0; i < data.costs.size(); ++i) {
    double peak = 0.0, total = 0.0;
    bool collision = false;
    for (size_t j = 0; j < steps; ++j) {
      const auto [risk, exact_collision] = poseRisk(data.trajectories.x(i, j), data.trajectories.y(i, j),
        data.trajectories.yaws(i, j), offset + (j + 1) * data.model_dt, relevant);
      peak = std::max(peak, risk); total += risk;
      collision |= exact_collision;
    }
    data.costs(i) += weight_ * (0.5 * peak + total / steps);
    if (collision) {
      data.costs(i) += collision_cost_;
    }
    colliding += collision;
  }
  data.fail_flag |= colliding == data.costs.size();
  if (colliding == data.costs.size()) {
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 1000,
      "TimedObstacleCritic all trajectories collide: %zu/%zu points=%zu cloud_age=%.3f",
      colliding, data.costs.size(), points_.size(), age);
  }
}
}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::TimedObstacleCritic, mppi::critics::CriticFunction)
