// Jazzy's Python MessageInfo omits publisher_gid; use the existing C++ ROS API.
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tf_publisher_probe");
  const double duration = node->declare_parameter("duration_s", 15.0);
  std::map<std::string, std::map<std::string, std::string>> edges;
  auto callback = [&](const std::string &topic) {
    return [&, topic](tf2_msgs::msg::TFMessage::ConstSharedPtr message,
                       const rclcpp::MessageInfo &info) {
      const auto &gid = info.get_rmw_message_info().publisher_gid;
      std::ostringstream identity;
      for (auto byte : gid.data)
        identity << std::hex << std::setw(2) << std::setfill('0') << int(byte);
      std::string owner = "unknown";
      for (const auto &endpoint : node->get_publishers_info_by_topic(topic)) {
        if (std::memcmp(endpoint.endpoint_gid().data(), gid.data,
                        RMW_GID_STORAGE_SIZE) == 0) {
          owner = endpoint.node_namespace();
          if (owner.empty() || owner.back() != '/') owner += '/';
          owner += endpoint.node_name();
          break;
        }
      }
      for (const auto &transform : message->transforms)
        edges[transform.header.frame_id + "->" + transform.child_frame_id]
             [identity.str()] = owner;
    };
  };
  auto dynamic = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100), callback("/tf"));
  auto fixed = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).transient_local(), callback("/tf_static"));
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cout << "{\"edges\":{";
  bool first_edge = true;
  for (const auto &[edge, publishers] : edges) {
    if (!first_edge) std::cout << ',';
    first_edge = false;
    std::cout << std::quoted(edge) << ":[";
    bool first_owner = true;
    for (const auto &[gid, owner] : publishers) {
      if (!first_owner) std::cout << ',';
      first_owner = false;
      std::cout << std::quoted(owner + '@' + gid);
    }
    std::cout << ']';
  }
  std::cout << "}}" << std::endl;
  rclcpp::shutdown();
  return 0;
}
