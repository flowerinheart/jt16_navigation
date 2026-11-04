#include "jt16_bringup/prediction_layer.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace jt16_bringup
{
namespace
{
bool validLayout(const sensor_msgs::msg::PointCloud2 & message)
{
  return message.height == 1 && !message.is_bigendian && message.point_step == 12 &&
         message.row_step == uint64_t(message.width) * 12 &&
         message.data.size() == message.row_step && message.fields.size() >= 2;
}
}  // namespace

void PredictionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("prediction layer node expired");}
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);
  current_ = !enabled_;
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/predicted_obstacles", rclcpp::QoS(1),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {receive(*message);}, options);
}

void PredictionLayer::receive(const sensor_msgs::msg::PointCloud2 & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  valid_ = false;
  points_.clear();
  if (message.header.frame_id.empty() || message.header.stamp.sec < 0 || !validLayout(message))
  {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 2000, "Invalid prediction cloud layout/frame");
    return;
  }
  for (size_t i = 0; i < 2; ++i) {
    const auto & field = message.fields[i];
    if (field.name != (i == 0 ? "x" : "y") || field.offset != 4 * i ||
      field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1)
    {
      return;
    }
  }
  for (size_t offset = 0; offset < message.data.size(); offset += 12) {
    float x, y;
    std::memcpy(&x, message.data.data() + offset, 4);
    std::memcpy(&y, message.data.data() + offset + 4, 4);
    if (!std::isfinite(x) || !std::isfinite(y)) {points_.clear(); return;}
    points_.emplace_back(x, y);
  }
  if (!transformPoints(message)) {points_.clear(); return;}
  stamp_ = rclcpp::Time(message.header.stamp, clock_->get_clock_type());
  valid_ = true;
}

bool PredictionLayer::transformPoints(const sensor_msgs::msg::PointCloud2 & message)
{
  if (message.header.frame_id == layered_costmap_->getGlobalFrameID()) {return true;}
  if (!tf_) {return false;}
  try {
    const auto transform = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), message.header.frame_id,
      rclcpp::Time(message.header.stamp));
    tf2::Transform frame;
    tf2::fromMsg(transform.transform, frame);
    for (auto & point : points_) {
      const auto translated = frame * tf2::Vector3(point.first, point.second, 0.0);
      point = {translated.x(), translated.y()};
    }
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Prediction transform failed: %s", error.what());
    return false;
  }
  return true;
}

bool PredictionLayer::fresh() const
{
  if (!valid_) {return false;}
  const double age = (clock_->now() - stamp_).seconds();
  return age >= -0.03 && age <= 0.5;
}

void PredictionLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  points_.clear();
  valid_ = false;
  current_ = !enabled_;
}

void PredictionLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_ = !enabled_ || fresh();
  auto * map = layered_costmap_->getCostmap();
  // Recompose the window so old forecasts cannot become permanent obstacles.
  *min_x = std::min(*min_x, map->getOriginX());
  *min_y = std::min(*min_y, map->getOriginY());
  *max_x = std::max(*max_x, map->getOriginX() + map->getSizeInCellsX() * map->getResolution());
  *max_y = std::max(*max_y, map->getOriginY() + map->getSizeInCellsY() * map->getResolution());
}

void PredictionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_ || !fresh()) {return;}
  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation;
  for (const auto & plugin : *layered_costmap_->getPlugins()) {
    inflation = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(plugin);
    if (inflation) {break;}
  }
  if (!inflation) {current_ = false; return;}
  const int radius = std::ceil(inflation->getInflationRadius() / master.getResolution());
  for (const auto & point : points_) {
    unsigned int x, y;
    if (!master.worldToMap(point.first, point.second, x, y)) {continue;}
    for (int row = std::max(min_j, int(y) - radius); row < std::min(max_j, int(y) + radius + 1); ++row) {
      for (int col = std::max(min_i, int(x) - radius); col < std::min(max_i, int(x) + radius + 1); ++col) {
        const double distance = std::hypot(col - int(x), row - int(y));
        if (distance > radius) {continue;}
        // Forecasts are uncertain future occupancy, not a hard collision at every time.
        const auto cost = std::min<unsigned char>(252, inflation->computeCost(distance));
        const auto old = master.getCost(col, row);
        master.setCost(col, row, old == nav2_costmap_2d::NO_INFORMATION ?
          cost : std::max(old, cost));
      }
    }
  }
}
}  // namespace jt16_bringup

PLUGINLIB_EXPORT_CLASS(jt16_bringup::PredictionLayer, nav2_costmap_2d::Layer)
