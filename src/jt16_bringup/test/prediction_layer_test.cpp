#include <chrono>
#include <cstring>
#include <limits>
#include <thread>
#include "gtest/gtest.h"
#include "jt16_bringup/prediction_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

class PredictionLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node = std::make_shared<nav2_util::LifecycleNode>("prediction_layer_test");
    input = std::make_shared<rclcpp::Node>("prediction_input_test");
    parent = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", false, false);
    parent->resizeMap(40, 40, .1, -2., -2.);
    layer = std::make_shared<jt16_bringup::PredictionLayer>();
    layer->initialize(parent.get(), "prediction", nullptr, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    parent->addPlugin(layer);
    auto inflation = std::make_shared<nav2_costmap_2d::InflationLayer>();
    inflation->initialize(parent.get(), "inflation", nullptr, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    parent->addPlugin(inflation);
    publisher = input->create_publisher<sensor_msgs::msg::PointCloud2>("/predicted_obstacles", 1);
    executor->add_node(node->get_node_base_interface());
    executor->add_node(input);
  }
  void TearDown() override {rclcpp::shutdown();}

  sensor_msgs::msg::PointCloud2 cloud()
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = "odom";
    message.height = 1; message.width = 1; message.point_step = 12; message.row_step = 12;
    for (int i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField field;
      field.name = i == 0 ? "x" : (i == 1 ? "y" : "z");
      field.offset = 4 * i; field.count = 1; field.datatype = field.FLOAT32;
      message.fields.push_back(field);
    }
    float xyz[3] = {1.f, 0.f, 0.f};
    message.data.resize(12);
    std::memcpy(message.data.data(), xyz, 12);
    return message;
  }

  bool deliver(sensor_msgs::msg::PointCloud2 message, bool current, unsigned char cost)
  {
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    do {
      message.header.stamp = node->now();
      publisher->publish(message);
      executor->spin_some();
      parent->updateMap(0., 0., 0.);
      if (layer->isCurrent() == current && parent->getCostmap()->getCost(30, 20) == cost) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (std::chrono::steady_clock::now() < end);
    return false;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::shared_ptr<rclcpp::Node> input;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> parent;
  std::shared_ptr<jt16_bringup::PredictionLayer> layer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
};

TEST_F(PredictionLayerTest, ForecastsClearWithoutErasingRealCostsAndRejectInvalidInput)
{
  auto message = cloud();
  ASSERT_TRUE(deliver(message, true, 252));
  EXPECT_GT(parent->getCostmap()->getCost(29, 20), nav2_costmap_2d::FREE_SPACE);
  parent->getCostmap()->setCost(15, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  parent->getCostmap()->setCost(30, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  layer->updateCosts(*parent->getCostmap(), 0, 0, 40, 40);
  EXPECT_EQ(parent->getCostmap()->getCost(15, 20), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(parent->getCostmap()->getCost(30, 20), nav2_costmap_2d::LETHAL_OBSTACLE);
  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  parent->updateMap(0., 0., 0.);
  EXPECT_FALSE(layer->isCurrent());
  EXPECT_EQ(parent->getCostmap()->getCost(30, 20), nav2_costmap_2d::FREE_SPACE);
  ASSERT_TRUE(deliver(message, true, 252));
  auto empty = message;
  empty.width = 0; empty.row_step = 0; empty.data.clear();
  ASSERT_TRUE(deliver(empty, true, nav2_costmap_2d::FREE_SPACE));
  message.header.frame_id = "map";
  ASSERT_TRUE(deliver(message, false, nav2_costmap_2d::FREE_SPACE));
  message = cloud();
  ASSERT_TRUE(deliver(message, true, 252));
  float invalid = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(message.data.data(), &invalid, 4);
  EXPECT_TRUE(deliver(message, false, nav2_costmap_2d::FREE_SPACE));
}
