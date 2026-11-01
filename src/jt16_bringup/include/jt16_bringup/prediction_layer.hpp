#pragma once

#include <mutex>
#include <utility>
#include <vector>
#include "nav2_costmap_2d/layer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace jt16_bringup
{
class PredictionLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void reset() override;
  bool isClearable() override {return true;}
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;

private:
  void receive(const sensor_msgs::msg::PointCloud2 &);
  bool fresh() const;
  std::mutex mutex_;
  std::vector<std::pair<float, float>> points_;
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  bool valid_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace jt16_bringup
