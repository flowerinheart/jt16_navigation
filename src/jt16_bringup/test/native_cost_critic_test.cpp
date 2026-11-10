#include <cmath>
#include "gtest/gtest.h"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

class NativeCostCritic : public mppi::critics::CostCritic
{
public:
  float threshold() {initialize(); return possible_collision_cost_;}
  bool collision(float cost, float x, float y, float yaw) {return inCollision(cost, x, y, yaw);}
};

TEST(NativeCostCriticTest, RasterizedFootprintAndChangingResolution)
{
  rclcpp::init(0, nullptr);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "native_bound_test", "", "native_bound_test", true);
  map->set_parameter(rclcpp::Parameter("footprint_padding", 0.0));
  map->set_parameter(rclcpp::Parameter("footprint", "[[.21,.245],[.21,-.245],[-.21,-.245],[-.21,.245]]"));
  map->declare_parameter("inflation_layer.inflation_radius", 0.75);
  map->declare_parameter("inflation_layer.cost_scaling_factor", 2.0);
  map->on_configure(rclcpp_lifecycle::State());
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x=.21; footprint[0].y=.245;
  footprint[1].x=.21; footprint[1].y=-.245;
  footprint[2].x=-.21; footprint[2].y=-.245;
  footprint[3].x=-.21; footprint[3].y=.245;
  map->setRobotFootprint(footprint);
  auto inflation = nav2_costmap_2d::InflationLayer::getInflationLayer(map, "");
  ASSERT_NE(inflation, nullptr);
  NativeCostCritic critic;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("native_critic_test");
  node->declare_parameter("critic.consider_footprint", true);
  mppi::ParametersHandler parameters(node);
  critic.on_configure(node, "mppi", "critic", map, &parameters);
  float previous = -1;
  size_t old_misses = 0;
  for (double resolution : {.05, .1, .025}) {
    map->getLayeredCostmap()->resizeMap(80, 80, resolution, -40*resolution, -40*resolution);
    auto grid = map->getCostmap();
    double obstacle_x, obstacle_y;
    grid->mapToWorld(40, 40, obstacle_x, obstacle_y);
    for (unsigned int y=0; y<80; ++y) {
      for (unsigned int x=0; x<80; ++x) {
        grid->setCost(x,y,inflation->computeCost(std::hypot(double(x)-40,double(y)-40)));
      }
    }
    const float threshold = critic.threshold();
    EXPECT_NE(threshold, previous);
    previous = threshold;
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> full(grid);
    nav2_smac_planner::GridCollisionChecker smac(map,32,node);
    smac.setFootprint(footprint,false,nav2_smac_planner::findCircumscribedCost(map));
    EXPECT_EQ(nav2_smac_planner::findCircumscribedCost(map), threshold);
    const float old_threshold = inflation->computeCost(std::hypot(.21,.245)/resolution);
    for (int angle=0; angle<32; ++angle) {
      const float yaw = float(angle)*float(2*3.141592653589793/32);
      for (int ix=-24; ix<=24; ++ix) {
        for (int iy=-24; iy<=24; ++iy) {
          const float x=obstacle_x+ix*resolution/3;
          const float y=obstacle_y+iy*resolution/3;
          const float gx=(x-grid->getOriginX())/resolution;
          const float gy=(y-grid->getOriginY())/resolution;
          const auto pose=nav2_smac_planner::getWorldCoords(gx,gy,grid);
          if (full.footprintCostAtPose(pose.position.x,pose.position.y,yaw,footprint)==254) {
            EXPECT_TRUE(smac.inCollision(gx,gy,float(angle),false));
          }
          if (full.footprintCostAtPose(x,y,yaw,footprint)!=254) {continue;}
          unsigned int mx,my;
          ASSERT_TRUE(grid->worldToMap(x,y,mx,my));
          const float center = grid->getCost(mx,my);
          if (center<old_threshold) {++old_misses;}
          EXPECT_TRUE(critic.collision(center,x,y,yaw));
        }
      }
    }
  }
  EXPECT_GT(old_misses, 0u);
  map->set_parameter(rclcpp::Parameter("inflation_layer.inflation_radius", 0.1));
  EXPECT_EQ(critic.threshold(), 0.0f);
  auto grid = map->getCostmap();
  grid->resetMap(0, 0, 80, 80);
  unsigned int mx,my;
  ASSERT_TRUE(grid->worldToMap(.21,.245,mx,my));
  grid->setCost(mx,my,254);
  mppi::models::State state;
  mppi::models::Trajectories trajectories;
  trajectories.reset(2,2);
  trajectories.x(1,0)=-.5f; trajectories.y(1,0)=-.5f;
  mppi::models::Path path;
  geometry_msgs::msg::Pose goal;
  goal.position.x=2;
  xt::xtensor<float,1> costs=xt::zeros<float>({2});
  float dt=.1;
  mppi::CriticData data{state,trajectories,path,goal,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};
  critic.score(data);
  EXPECT_GT(costs(0), 0.0f);
  EXPECT_EQ(costs(1), 0.0f);
  EXPECT_FALSE(data.fail_flag);
  inflation.reset();
  map->on_cleanup(rclcpp_lifecycle::State());
  map.reset();
  rclcpp::shutdown();
}



class RetryOptimizer : public mppi::Optimizer
{
public:
  void fail()
  {
    settings_.batch_size=2;settings_.time_steps=4;settings_.model_dt=.1;
    settings_.sampling_std.vx=.2;settings_.sampling_std.wz=.4;
    motion_model_=std::make_shared<mppi::DiffDriveMotionModel>();
    critics_data_.fail_flag=true;
  }
  bool failed() const {return critics_data_.fail_flag;}
};

TEST(NativeCostCriticTest, ResetClearsFailureBeforeRetry)
{
  rclcpp::init(0,nullptr);
  RetryOptimizer optimizer;optimizer.fail();
  optimizer.reset(false);
  EXPECT_FALSE(optimizer.failed());
  rclcpp::shutdown();
}
