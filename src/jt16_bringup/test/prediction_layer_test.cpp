#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>
#include "gtest/gtest.h"
#include "jt16_bringup/prediction_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "tf2_ros/buffer.h"

class PredictionLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node = std::make_shared<nav2_util::LifecycleNode>("prediction_layer_test");
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    input = std::make_shared<rclcpp::Node>("prediction_input_test");
    parent = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", false, false);
    parent->resizeMap(40, 40, .1, -2., -2.);
    layer = std::make_shared<jt16_bringup::PredictionLayer>();
    layer->initialize(parent.get(), "prediction", buffer.get(), node,
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
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> parent;
  std::shared_ptr<jt16_bringup::PredictionLayer> layer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
};

TEST_F(PredictionLayerTest, SameCellForecastsPreserveTheWholeGridAcrossUpdatesAndOrigins)
{
  auto message = cloud();
  ASSERT_TRUE(deliver(message, true, 252));
  nav2_costmap_2d::Costmap2D single(40, 40, .1, -2., -2.);
  layer->updateCosts(single, 0, 0, 40, 40);
  const float extra[6] = {1.02f, .02f, 0.f, 1.08f, .08f, 0.f};
  message.width = 3; message.row_step = 36; message.data.resize(36);
  std::memcpy(message.data.data() + 12, extra, sizeof(extra));
  ASSERT_TRUE(deliver(message, true, 252));
  nav2_costmap_2d::Costmap2D multiple(40, 40, .1, -2., -2.);
  layer->updateCosts(multiple, 0, 0, 40, 40);
  EXPECT_EQ(std::memcmp(single.getCharMap(), multiple.getCharMap(), 1600), 0);
  multiple.resetMap(0, 0, 40, 40);
  layer->updateCosts(multiple, 0, 0, 40, 40);
  EXPECT_EQ(std::memcmp(single.getCharMap(), multiple.getCharMap(), 1600), 0);
  nav2_costmap_2d::Costmap2D shifted(40, 40, .1, -1.95, -1.95);
  layer->updateCosts(shifted, 0, 0, 40, 40);
  EXPECT_EQ(shifted.getCost(29, 19), 252);
  EXPECT_EQ(shifted.getCost(30, 20), 252);
  message.width = 2; message.row_step = 24;
  std::memcpy(message.data.data() + 12, extra + 3, 12);
  message.data.resize(24);
  ASSERT_TRUE(deliver(message, true, 252));
  nav2_costmap_2d::Costmap2D reference(40, 40, .1, -1.95, -1.95);
  layer->updateCosts(reference, 0, 0, 40, 40);
  EXPECT_EQ(std::memcmp(shifted.getCharMap(), reference.getCharMap(), 1600), 0);
}

TEST_F(PredictionLayerTest, ForecastsTransformIntoTheCostmapFrame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "map";
  transform.transform.translation.x = 2.0;
  transform.transform.rotation.z = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);
  ASSERT_TRUE(buffer->setTransform(transform, "test", true));
  auto message = cloud();
  message.header.frame_id = "map";
  const float xyz[3] = {0.f, 1.f, 0.f};
  std::memcpy(message.data.data(), xyz, 12);
  ASSERT_TRUE(deliver(message, true, 252));
  EXPECT_EQ(parent->getCostmap()->getCost(20, 30), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(PredictionLayerTest, ForecastsCoverUnknownCellsAndExpireWithoutErasingLethalCosts)
{
  auto * map = parent->getCostmap();
  map->setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  map->resetMap(0, 0, 40, 40);
  ASSERT_TRUE(deliver(cloud(), true, 252));
  EXPECT_EQ(map->getCost(0, 0), nav2_costmap_2d::NO_INFORMATION);
  map->setCost(30, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  layer->updateCosts(*map, 0, 0, 40, 40);
  EXPECT_EQ(map->getCost(30, 20), nav2_costmap_2d::LETHAL_OBSTACLE);
  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  parent->updateMap(0., 0., 0.);
  EXPECT_FALSE(layer->isCurrent());
  EXPECT_EQ(map->getCost(30, 20), nav2_costmap_2d::NO_INFORMATION);
}

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

TEST_F(PredictionLayerTest, ConfiguredDisabledLayerLeavesBoundsAndCostsUnchanged)
{
  node->declare_parameter("disabled_prediction.enabled", false);
  auto disabled=std::make_shared<jt16_bringup::PredictionLayer>();
  disabled->initialize(parent.get(), "disabled_prediction", buffer.get(), node,
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  EXPECT_TRUE(disabled->isCurrent());
  double min_x=.3,min_y=.4,max_x=.5,max_y=.6;
  disabled->updateBounds(0,0,0,&min_x,&min_y,&max_x,&max_y);
  EXPECT_EQ(min_x,.3);EXPECT_EQ(min_y,.4);EXPECT_EQ(max_x,.5);EXPECT_EQ(max_y,.6);
  nav2_costmap_2d::Costmap2D grid(40,40,.1,-2,-2,42);
  disabled->updateCosts(grid,0,0,40,40);
  EXPECT_TRUE(std::all_of(grid.getCharMap(),grid.getCharMap()+1600,
    [](unsigned char cost) {return cost==42;}));
  disabled->reset();
  EXPECT_TRUE(disabled->isCurrent());
  disabled->updateBounds(0,0,0,&min_x,&min_y,&max_x,&max_y);
  EXPECT_EQ(min_x,.3);EXPECT_EQ(min_y,.4);EXPECT_EQ(max_x,.5);EXPECT_EQ(max_y,.6);
  disabled->updateCosts(grid,0,0,40,40);
  EXPECT_TRUE(std::all_of(grid.getCharMap(),grid.getCharMap()+1600,
    [](unsigned char cost) {return cost==42;}));
}
