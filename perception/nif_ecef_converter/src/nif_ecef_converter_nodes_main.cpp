//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by USRG on 9/14/21.
//
#include "ekf_localizer/nif_ecef_converter_node.h"

#include "rcutils/error_handling.h"
#include <cstdio>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // using nif::perception::LCfusionNode;

  auto nd = std::make_shared<ECEFConverterNode>();

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