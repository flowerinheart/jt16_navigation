cmake_minimum_required(VERSION 3.16)
project(jt16_bringup)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
add_library(jt16_timed_obstacle_critic SHARED src/timed_obstacle_critic.cpp)
target_compile_features(jt16_timed_obstacle_critic PUBLIC cxx_std_17)
target_include_directories(jt16_timed_obstacle_critic PUBLIC include)
ament_target_dependencies(jt16_timed_obstacle_critic nav2_mppi_controller sensor_msgs rclcpp)
pluginlib_export_plugin_description_file(nav2_mppi_controller timed_critic_plugins.xml)
install(TARGETS jt16_timed_obstacle_critic LIBRARY DESTINATION lib)
add_library(jt16_prediction_layer SHARED src/prediction_layer.cpp)
target_compile_features(jt16_prediction_layer PUBLIC cxx_std_17)
target_include_directories(jt16_prediction_layer PUBLIC include)
ament_target_dependencies(jt16_prediction_layer nav2_costmap_2d pluginlib sensor_msgs rclcpp)
pluginlib_export_plugin_description_file(nav2_costmap_2d prediction_plugins.xml)
install(TARGETS jt16_prediction_layer LIBRARY DESTINATION lib)
add_executable(tf_publisher_probe src/tf_publisher_probe.cpp)
target_compile_features(tf_publisher_probe PRIVATE cxx_std_17)
ament_target_dependencies(tf_publisher_probe rclcpp tf2_msgs)
install(TARGETS tf_publisher_probe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/scripts/sensor_geometry.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/scripts/benchmark_geometry.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/scripts/instant_cloud_adapter.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/scripts/project_nav_scan.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/scripts/sim_imu_adapter.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/scripts/lio_odom_bridge.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/scripts/motion_prediction.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/scripts/predict_obstacles.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(prediction_layer_test test/prediction_layer_test.cpp)
  target_link_libraries(prediction_layer_test jt16_prediction_layer)
  ament_target_dependencies(prediction_layer_test nav2_costmap_2d sensor_msgs rclcpp)
  ament_add_gtest(timed_obstacle_critic_test test/timed_obstacle_critic_test.cpp)
  ament_target_dependencies(timed_obstacle_critic_test nav2_mppi_controller sensor_msgs rclcpp pluginlib)
endif()
ament_package()
