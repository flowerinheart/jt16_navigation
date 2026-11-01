#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>
#include "gtest/gtest.h"
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class TimedCriticTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<nav2_util::LifecycleNode>("timed_critic_test");
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("timed_costmap", false);
    costmap->set_parameters({rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("footprint", "[[0.20,0.235],[0.20,-0.235],[-0.20,-0.235],[-0.20,0.235]]")});
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    parameters = std::make_unique<mppi::ParametersHandler>(node);
    loader = std::make_unique<pluginlib::ClassLoader<mppi::critics::CriticFunction>>(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
    critic = loader->createSharedInstance("mppi::critics::TimedObstacleCritic");
    critic->on_configure(node, "FollowPath", "FollowPath.TimedObstacleCritic", costmap, parameters.get());
    input = std::make_shared<rclcpp::Node>("timed_critic_input");
    publisher = input->create_publisher<sensor_msgs::msg::PointCloud2>("/moving_obstacle_points", 1);
    trajectories.reset(2, 30);
    costs = xt::zeros<float>({2});
    data = std::make_unique<mppi::CriticData>(mppi::CriticData{
      state, trajectories, path, goal, costs, dt, false, nullptr, nullptr, std::nullopt, std::nullopt});
    state.pose.header.frame_id = "odom";
    for (size_t j = 0; j < 30; ++j) {
      trajectories.x(0, j) = 2 * (j + 1) * dt;
      trajectories.x(1, j) = 1.0;
    }
  }

  void TearDown() override
  {
    critic.reset();
    costmap->on_cleanup(rclcpp_lifecycle::State());
    rclcpp::shutdown();
  }

  sensor_msgs::msg::PointCloud2 cloud()
  {
    sensor_msgs::msg::PointCloud2 m;
    m.header.frame_id = "odom"; m.height = 1; m.width = 1;
    m.point_step = 16; m.row_step = 16;
    const std::array<std::string, 4> names{"x", "y", "vx", "vy"};
    for (size_t i = 0; i < names.size(); ++i) {
      sensor_msgs::msg::PointField f;
      f.name = names[i]; f.offset = 4 * i; f.count = 1; f.datatype = f.FLOAT32;
      m.fields.push_back(f);
    }
    const float values[4] = {1.0f, 1.0f, 0.0f, -0.5f};
    m.data.resize(16); std::memcpy(m.data.data(), values, 16);
    return m;
  }

  void send(sensor_msgs::msg::PointCloud2 m, double age = 0.0)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!publisher->get_subscription_count() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(publisher->get_subscription_count(), 0U);
    state.pose.header.stamp = node->now();
    m.header.stamp = rclcpp::Time(state.pose.header.stamp) - rclcpp::Duration::from_seconds(age);
    for (int i = 0; i < 10; ++i) {
      publisher->publish(m);
      rclcpp::spin_some(node->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    costs.fill(0.0f); data->fail_flag = false;
    critic->score(*data);
  }

  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::shared_ptr<rclcpp::Node> input;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::unique_ptr<mppi::ParametersHandler> parameters;
  std::unique_ptr<pluginlib::ClassLoader<mppi::critics::CriticFunction>> loader;
  std::shared_ptr<mppi::critics::CriticFunction> critic;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  mppi::models::State state;
  mppi::models::Trajectories trajectories;
  mppi::models::Path path;
  geometry_msgs::msg::Pose goal;
  xt::xtensor<float, 1> costs;
  float dt{0.1f};
  std::unique_ptr<mppi::CriticData> data;
};

TEST_F(TimedCriticTest, PassBeforeArrivalIsSafeButWaitingForLateralArrivalIsNot)
{
  send(cloud());
  ASSERT_FALSE(data->fail_flag);
  EXPECT_NEAR(costs(0), 0.0f, 1e-6);
  EXPECT_GT(costs(1), 50.0f);
}

TEST_F(TimedCriticTest, EmptyCloudIsFreshAndOldOrInvalidCloudFails)
{
  auto m = cloud(); m.width = 0; m.row_step = 0; m.data.clear();
  send(m);
  EXPECT_FALSE(data->fail_flag); EXPECT_EQ(costs(1), 0.0f);
  send(m, 0.6); EXPECT_TRUE(data->fail_flag);
  send(m, -1.0); EXPECT_TRUE(data->fail_flag);
  m = cloud(); m.header.frame_id = "map";
  send(m); EXPECT_TRUE(data->fail_flag);
  m = cloud(); const float bad = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(m.data.data(), &bad, 4);
  send(m); EXPECT_TRUE(data->fail_flag);
}

TEST_F(TimedCriticTest, TimestampOffsetAndRobotOrientationAreUsed)
{
  trajectories.reset(2, 5); trajectories.x.fill(1.0f);
  auto m = cloud(); const float approaching[4] = {1.0f, .85f, 0.0f, -.5f};
  std::memcpy(m.data.data(), approaching, 16);
  send(m);
  ASSERT_FALSE(data->fail_flag); EXPECT_NEAR(costs(0), 0.0f, 1e-6);
  send(m, 0.3);
  ASSERT_FALSE(data->fail_flag);
  EXPECT_GT(costs(0), 1.0f);
  trajectories.reset(2, 30);
  trajectories.x.fill(0.0f); trajectories.y.fill(0.0f);
  m = cloud(); const float p[4] = {.52f, 0.0f, .08f, 0.0f};
  std::memcpy(m.data.data(), p, 16);
  send(m); const float forward = costs(0);
  trajectories.yaws.fill(1.57079632679f);
  send(m);
  EXPECT_GT(costs(0), forward);
}

TEST_F(TimedCriticTest, NearestSurfaceMatchesOriginalPerPointRisk)
{
  trajectories.reset(2, 30);
  auto m = cloud();
  m.width = 32; m.row_step = 16 * m.width; m.data.resize(m.row_step);
  for (size_t k = 0; k < m.width; ++k) {
    const float point[4] = {float(k) * .025f, .1f + float(k) * .012f, 0.f, 0.f};
    std::memcpy(m.data.data() + 16 * k, point, 16);
  }
  double peak = 0., total = 0.;
  double half_x = 0., half_y = 0.;
  for (const auto & p : costmap->getRobotFootprint()) {
    half_x = std::max(half_x, std::abs(p.x));
    half_y = std::max(half_y, std::abs(p.y));
  }
  for (size_t j = 0; j < 30; ++j) {
    const float x = float(j) * .018f, y = -.1f, yaw = float(j) * .04f;
    trajectories.x(0, j) = x; trajectories.y(0, j) = y; trajectories.yaws(0, j) = yaw;
    double worst = 0.;
    for (size_t k = 0; k < m.width; ++k) {
      float p[4]; std::memcpy(p, m.data.data() + 16 * k, 16);
      const double dx = p[0] - x, dy = p[1] - y;
      const double bx = std::cos(yaw) * dx + std::sin(yaw) * dy;
      const double by = -std::sin(yaw) * dx + std::cos(yaw) * dy;
      const double gap = std::hypot(std::max(std::abs(bx) - half_x, 0.),
        std::max(std::abs(by) - half_y, 0.));
      const double risk = std::max(0., 1. - gap / double(.35f));
      worst = std::max(worst, risk * risk);
    }
    peak = std::max(peak, worst); total += worst;
  }
  send(m);
  ASSERT_FALSE(data->fail_flag);
  EXPECT_NEAR(costs(0), 100. * (.5 * peak + total / 30), 1e-4);
}
