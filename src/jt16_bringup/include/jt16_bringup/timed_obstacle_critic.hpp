#pragma once

#include <array>
#include <mutex>
#include <vector>
#include <utility>
#include "nav2_mppi_controller/critic_function.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mppi::critics
{
class TimedObstacleCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

private:
  void receive(const sensor_msgs::msg::PointCloud2 & message);
  std::pair<double, bool> poseRisk(double x, double y, double yaw, double horizon,
    const std::vector<std::array<float, 4>> & points) const;
  std::mutex mutex_;
  std::vector<std::array<float, 4>> points_;
  std::array<double, 4> bounds_;
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  bool valid_{false};
  float weight_{100.0f}, clearance_{0.35f}, collision_cost_{1000000.0f};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace mppi::critics
