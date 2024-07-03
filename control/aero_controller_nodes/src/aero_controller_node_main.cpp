#include "aero_controller_nodes/aero_controller_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // using nif::perception::LCfusionNode;

  auto nd = std::make_shared<AeroControlNode>();

  // const char *node_name = "Lidar_and_camera_fusion_node";

  // rclcpp::Node::SharedPtr nd;

  // try {
  //   RCLCPP_INFO(this->get_logger(),
  //               "Instantiating LCfusionNode with name: %s",
  //               node_name);
  //   rclcpp::NodeOptions options;

  //   nd = std::make_shared<LCfusionNode>();
  // } catch (std::exception &e) {
  //   RCLCPP_FATAL(this->get_logger(),
  //                "FATAL ERROR during node initialization: ABORTING.\n%s",
  //                e.what());
  //   return -1;
  // }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  // RCLCPP_INFO(this->get_logger(),
  //             "Shutting down %s [LCfusionNode]", node_name);

  return 0;
}