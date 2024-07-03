

//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_dynamic_planning_nodes/dynamic_planning_node_pit.h"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdlib.h>

using namespace nif::planning;
using namespace std;
DynamicPlannerNode::DynamicPlannerNode(const std::string &node_name_)
    : IBaseNode(node_name_, common::NodeType::PLANNING)
{
  this->setNodeStatus(nif::common::NODE_NOT_INITIALIZED);

  m_config_load_success = false;

  std::string package_share_directory;
  try
  {
    // This value shouldn't be used, it's as a backup if a config param is
    // missing.
    package_share_directory = ament_index_cpp::get_package_share_directory(
        "nif_dynamic_planning_nodes");
  }
  catch (std::exception e)
  {
    RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
  }
  package_share_directory = package_share_directory.append("/");

  this->declare_parameter("planning_config_file_path", "");
  this->declare_parameter("velocity_profile_config_file_path", "");
  this->declare_parameter("acc_config_file_path", "");
  this->declare_parameter("maps_path_root", "");
  this->declare_parameter("vis_flg", true);
  this->declare_parameter("TRACK", "");
  this->declare_parameter("dist_same_lane_path_change", 4.0);

  this->get_parameter("maps_path_root", this->m_map_root_path);
  this->m_map_root_path.append("/");
  m_planning_config_file_path =
      this->get_parameter("planning_config_file_path").as_string();
  m_velocity_profile_config_file_path =
      this->get_parameter("velocity_profile_config_file_path").as_string();
  m_acc_config_file_path =
      this->get_parameter("acc_config_file_path").as_string();
  m_vis_flg = this->get_parameter("vis_flg").as_bool();
  std::string TRACK = this->get_parameter("TRACK").as_string();
  m_dist_same_lane_path_change = this->get_parameter("dist_same_lane_path_change").as_double();
  if (m_planning_config_file_path.empty())
  {
    throw std::runtime_error(
        "Parameter m_planning_config_file_path not declared, or empty.");
  }
  if (m_velocity_profile_config_file_path.empty())
  {
    throw std::runtime_error("Parameter m_velocity_profile_config_file_path "
                             "not declared, or empty.");
  }

  YAML::Node config = YAML::LoadFile(m_acc_config_file_path);
  YAML::Node acc_config_param = config["acc_config_param"];
  m_small_gap = acc_config_param["small_gap"].as<double>();
  // std::cout << "small_gap "<< m_small_gap << std::endl;
  // Load param
  loadConfig(m_planning_config_file_path);
  m_config_load_success = true;
  m_det_callback_first_run = true;
  m_oppo_pred_callback_first_run = true;
  m_timer_callback_first_run = true;

  // //////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR PIT LINE
  // //////////////////////////////////////////////
  m_pitline_file_path = this->m_map_root_path + m_pitline_file_path;
  auto pitline_xy = loadCSVfile(m_pitline_file_path);
  auto pitline_x_vec = get<0>(pitline_xy);
  auto pitline_y_vec = get<1>(pitline_xy);
  m_pitline_spline_data = m_frenet_generator_ptr->apply_cubic_spliner(
      pitline_x_vec, pitline_y_vec, m_config_spline_interval);
  m_pitline_x_vec = get<0>(m_pitline_spline_data);
  m_pitline_y_vec = get<1>(m_pitline_spline_data);
  m_pitline_path = xyyawVec2Path(get<0>(m_pitline_spline_data),
                                 get<1>(m_pitline_spline_data),
                                 get<2>(m_pitline_spline_data));

  if (pitline_x_vec.size() != pitline_y_vec.size() ||
      pitline_x_vec.size() == 0)
  {
    throw std::runtime_error(
        "STAY BEHIND wpt file has a problem. Stop node initialization.");
  }
  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] Pitline is loaded...");
  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] Loading Race line...");
  // //////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR RACING LINE
  // //////////////////////////////////////////////
  m_racingline_file_path = this->m_map_root_path + m_racingline_file_path;
  auto racingline_xy = loadCSVfile(m_racingline_file_path);
  auto racingline_x_vec = get<0>(racingline_xy);
  auto racingline_y_vec = get<1>(racingline_xy);
  m_racingline_spline_data = m_frenet_generator_ptr->apply_cubic_spliner(
      racingline_x_vec, racingline_y_vec, m_config_spline_interval);
  m_race_mode_spline_data = m_racingline_spline_data;
  m_racingline_x_vec = get<0>(m_racingline_spline_data);
  m_racingline_y_vec = get<1>(m_racingline_spline_data);
  m_racingline_path = xyyawVec2Path(get<0>(m_racingline_spline_data),
                                    get<1>(m_racingline_spline_data),
                                    get<2>(m_racingline_spline_data));
  m_race_mode_path = m_racingline_path;
  // minimal checking
  if (racingline_x_vec.size() != racingline_y_vec.size() ||
      racingline_x_vec.size() == 0)
  {
    throw std::runtime_error(
        "Racing wpt file has a problem. Stop node initialization.");
  }

  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] Raceline is loaded...");
  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] Loading defender line...");

  // ////////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR DEFENDER LINE
  // ////////////////////////////////////////////////
  m_defenderline_file_path = this->m_map_root_path + m_defenderline_file_path;
  auto defenderline_xy = loadCSVfile(m_defenderline_file_path);
  auto defenderline_x_vec = get<0>(defenderline_xy);
  auto defenderline_y_vec = get<1>(defenderline_xy);
  m_defenderline_spline_data = m_frenet_generator_ptr->apply_cubic_spliner(
      defenderline_x_vec, defenderline_y_vec, m_config_spline_interval);
  m_defenderline_x_vec = get<0>(m_defenderline_spline_data);
  m_defenderline_y_vec = get<1>(m_defenderline_spline_data);
  m_defenderline_path = xyyawVec2Path(get<0>(m_defenderline_spline_data),
                                      get<1>(m_defenderline_spline_data),
                                      get<2>(m_defenderline_spline_data));
  // minimal checking
  if (defenderline_x_vec.size() != defenderline_y_vec.size() ||
      defenderline_x_vec.size() == 0)
  {
    throw std::runtime_error(
        "Defender wpt file has a problem. Stop node initialization.");
  }

  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Defender line is loaded...");
  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Loading stay behind path...");

  // ////////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR STAYBEHIND LINE
  // ////////////////////////////////////////////////
  m_staybehind_file_path = this->m_map_root_path + m_staybehind_file_path;
  rclcpp::Time time1 = this->now();
  auto staybehind_xy = loadCSVfile(m_staybehind_file_path);
  rclcpp::Time time2 = this->now();
  rclcpp::Duration diff1 = time2 - time1;
  std::cout<< "diff1: "<< nif::common::utils::time::secs(diff1)<<std::endl;
  auto staybehind_x_vec = get<0>(staybehind_xy);
  auto staybehind_y_vec = get<1>(staybehind_xy);
  m_staybehind_spline_data = m_frenet_generator_ptr->apply_cubic_spliner(
      staybehind_x_vec, staybehind_y_vec, m_config_spline_interval);
  rclcpp::Time time3 = this->now();
  rclcpp::Duration diff2 = time3 - time2;
  std::cout<< "diff2: "<< nif::common::utils::time::secs(diff2)<<std::endl;
  m_staybehind_x_vec = get<0>(m_staybehind_spline_data);
  m_staybehind_y_vec = get<1>(m_staybehind_spline_data);
  m_staybehind_path = xyyawVec2Path(get<0>(m_staybehind_spline_data),
                                    get<1>(m_staybehind_spline_data),
                                    get<2>(m_staybehind_spline_data));
  rclcpp::Time time4 = this->now();
  rclcpp::Duration diff3 = time4 - time3;
  std::cout<< "diff3: "<< nif::common::utils::time::secs(diff3)<<std::endl;

  // minimal checking
  if (staybehind_x_vec.size() != staybehind_y_vec.size() ||
      staybehind_x_vec.size() == 0)
  {
    throw std::runtime_error(
        "STAY BEHIND wpt file has a problem. Stop node initialization.");
  }

  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Stay behind line is loaded...");
  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Loading center path...");

  // ////////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR CENTER LINE
  // ////////////////////////////////////////////////
  m_center_file_path = this->m_map_root_path + m_center_file_path;
  auto center_xy = loadCSVfile(m_center_file_path);
  auto center_x_vec = get<0>(center_xy);
  auto center_y_vec = get<1>(center_xy);
  m_center_spline_data = m_frenet_generator_ptr->apply_cubic_spliner(
      center_x_vec, center_y_vec, m_config_spline_interval);
  m_center_x_vec = get<0>(m_center_spline_data);
  m_center_y_vec = get<1>(m_center_spline_data);
  m_center_path = xyyawVec2Path(get<0>(m_center_spline_data),
                                get<1>(m_center_spline_data),
                                get<2>(m_center_spline_data));

  // minimal checking
  if (center_x_vec.size() != center_y_vec.size() ||
      center_x_vec.size() == 0)
  {
    throw std::runtime_error(
        "CETNER LINE wpt file has a problem. Stop node initialization.");
  }

  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Center line is loaded...");
  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Loading overtaking path candidates...");

  // //////////////////////////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR EVERY OVERTAKING PATH CANDIDATES
  // //////////////////////////////////////////////////////////////////
  for (int candidate_idx = 0; candidate_idx < m_num_overtaking_candidates;
       candidate_idx++)
  {
    m_overtaking_candidates_file_path_vec[candidate_idx] =
        this->m_map_root_path +
        m_overtaking_candidates_file_path_vec[candidate_idx];
    auto wpt_xy =
        loadCSVfile(m_overtaking_candidates_file_path_vec[candidate_idx]);
    auto path_x_vec = get<0>(wpt_xy);
    auto path_y_vec = get<1>(wpt_xy);
    auto splined_result = m_frenet_generator_ptr->apply_cubic_spliner(
        path_x_vec, path_y_vec, m_config_spline_interval);
    m_overtaking_candidates_spline_data_vec.push_back(splined_result);
    m_overtaking_candidates_spline_model_vec.push_back(get<4>(splined_result));
    auto candidate_path = xyyawVec2Path(
        get<0>(splined_result), get<1>(splined_result), get<2>(splined_result));
    m_overtaking_candidates_path_vec.push_back(candidate_path);

    // minimal checking
    if (path_x_vec.size() != path_y_vec.size() || path_x_vec.size() == 0)
    {
      throw std::runtime_error("One of the overtaking path file has a problem. "
                               "Stop node initialization.");
    }
  }

  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] All paths are loaded...");

  // INITIALIZE SUBSCRIBERS & PUBLISHER
  m_det_sub =
      this->create_subscription<nif::common::msgs::PerceptionResultList>(
          "tracking_output_topic_name", common::constants::QOS_PLANNING,
          std::bind(&DynamicPlannerNode::detectionResultCallback, this,
                    std::placeholders::_1));
  m_oppo_pred_sub = this->create_subscription<nif_msgs::msg::DynamicTrajectory>(
      "prediction_output_topic_name", common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNode::predictionResultCallback, this,
                std::placeholders::_1));
  m_oppo_refline_sub = this->create_subscription<nif_msgs::msg::OpponentRefline>(
      "opponent_prediction/oppo_ref_line", common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNode::opponentReferenceCallback, this,
                std::placeholders::_1));
  m_ego_traj_body_pub =
      this->create_publisher<nif_msgs::msg::DynamicTrajectory>(
          "out_trajectory_body", common::constants::QOS_PLANNING);
  m_ego_traj_global_pub =
      this->create_publisher<nif_msgs::msg::DynamicTrajectory>(
          "out_trajectory_global", common::constants::QOS_PLANNING);
  m_ego_traj_body_vis_pub = this->create_publisher<nav_msgs::msg::Path>(
      "out_trajectory_vis_body", common::constants::QOS_PLANNING);
  m_ego_traj_global_vis_pub = this->create_publisher<nav_msgs::msg::Path>(
      "out_trajectory_vis_global", common::constants::QOS_PLANNING);
  m_debug_vis_pub = this->create_publisher<nav_msgs::msg::Path>(
      "planning/debug", common::constants::QOS_PLANNING);

  m_ego_traj_global_vis_debug_pub1 =
      this->create_publisher<nav_msgs::msg::Path>(
          "planning/debug1", common::constants::QOS_PLANNING);
  m_ego_traj_global_debug_pub1 =
      this->create_publisher<nif_msgs::msg::DynamicTrajectory>(
          "planning/traj/debug1", common::constants::QOS_PLANNING);
  m_ego_traj_global_vis_debug_pub2 =
      this->create_publisher<nav_msgs::msg::Path>(
          "planning/debug2", common::constants::QOS_PLANNING);
  m_ego_traj_global_vis_debug_pub3 =
      this->create_publisher<nav_msgs::msg::Path>(
          "planning/debug3", common::constants::QOS_PLANNING);

  m_planner_status_pub =
      this->create_publisher<nif_msgs::msg::PlannerStatus>(
          "planning/status", common::constants::QOS_PLANNING);

  m_planner_timer = this->create_wall_timer(
      20ms, std::bind(&DynamicPlannerNode::timer_callback_rule, this)); // 50 hz

  // from turn 2 exit to turn 3 entry
  if (TRACK == "TMS")
  {
    this->m_backstretch_zone[0] = 4;
    this->m_backstretch_zone[1] = 6;
  }
  else if (TRACK == "LVMS_SIM")
  {
    this->m_backstretch_zone[0] = 3;
    this->m_backstretch_zone[1] = 5;
  }
  else if (TRACK == "LVMS")
  {
    this->m_backstretch_zone[0] = 4;
    this->m_backstretch_zone[1] = 6;
  }
  else if (TRACK == "LOR")
  {
    this->m_backstretch_zone[0] = 100;
    this->m_backstretch_zone[1] = 100;
  }
  else if (TRACK == "LVMS_SIM_AC")
  {
    this->m_backstretch_zone[0] = 3;
    this->m_backstretch_zone[1] = 6;
  }
  else if (TRACK == "MONZA")
  {
    this->m_backstretch_zone[0] = 3;
    this->m_backstretch_zone[1] = 6;
  }
  else if (TRACK == "MONZA_SIM")
  {
    this->m_backstretch_zone[0] = 3;
    this->m_backstretch_zone[1] = 6;
  }
  else
  {
    throw std::runtime_error("[Dynamic planner] Wrong TRACK name!");
  }

  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] Initialization done.");

  this->setNodeStatus(nif::common::NODE_INITIALIZED);
  m_last_update_target_path_alias = m_pitline_file_path;
}

void DynamicPlannerNode::loadConfig(const std::string &planning_config_file_)
{
  RCLCPP_INFO(get_logger(), "Loading planning params: %s",
              planning_config_file_.c_str());

  YAML::Node config = YAML::LoadFile(planning_config_file_);

  if (!config["path_candidates_param"])
  {
    throw std::runtime_error(
        "path_candidates_param field not defined in config file.");
  }
  if (!config["planning_params"])
  {
    throw std::runtime_error(
        "planning_params field not defined in config file.");
  }
  if (!config["collision_checking_params"])
  {
    throw std::runtime_error(
        "collision_checking_params field not defined in config file.");
  }
  if (!config["waypoint_manager_param"])
  {
    throw std::runtime_error(
        "waypoint_manager_param field not defined in config file.");
  }

  // path_candidates_param
  YAML::Node path_candidates_params = config["path_candidates_param"];
  // There might be duplicate file names, it is ok
  m_pitline_file_path =
      path_candidates_params["pitlane_path"].as<std::string>();
  m_racingline_file_path =
      path_candidates_params["racingline_path"].as<std::string>();
  m_race_mode_file_path = m_racingline_file_path;
  m_staybehind_file_path =
      path_candidates_params["staybehindline_path"].as<std::string>();
  m_defenderline_file_path =
      path_candidates_params["defenderline_path"].as<std::string>();
  m_center_file_path =
      path_candidates_params["centerline_path"].as<std::string>();
  m_overtaking_candidates_file_path_vec =
      path_candidates_params["overtaking_candidate_path_array"]
          .as<std::vector<std::string>>();
  m_overtaking_candidates_alias_vec =
      path_candidates_params["overtaking_candidate_path_alias_array"]
          .as<std::vector<std::string>>();

  // Size check (file_path - path_alias)
  if (m_overtaking_candidates_file_path_vec.size() !=
      m_overtaking_candidates_alias_vec.size())
  {
    throw std::runtime_error(
        "path_candidates_param is not properly set. Check config file.");
  }

  m_num_overtaking_candidates = m_overtaking_candidates_file_path_vec.size();

  // planning params
  YAML::Node planning_params = config["planning_params"];

  m_config_spline_interval = planning_params["splining_interval"].as<double>(); // Use 1m as default
  m_config_follow_enable_dist =
      planning_params["follow_enable_dist"].as<double>();
  m_config_planning_horizon =
      planning_params["planning_horizon_t"].as<double>();
  m_config_planning_dt = planning_params["planning_dt"].as<double>();
  m_config_max_accel = planning_params["max_accel"].as<double>();
  m_config_overtaking_longitudinal_margin =
      planning_params["overtaking_longitudinal_margin"].as<double>();
  m_config_overtaking_lateral_margin =
      planning_params["overtaking_lateral_margin"].as<double>();
  m_config_merging_longitudinal_margin =
      planning_params["merging_longitudinal_margin"].as<double>();
  m_config_merge_allow_dist =
      planning_params["merging_allow_dist_to_racingline"].as<double>();

  // collision_checking_params
  YAML::Node collision_checking_params = config["collision_checking_params"];

  m_config_overlap_checking_dist_bound =
      collision_checking_params["overlap_checking_dist_bound"].as<double>();
  m_config_overlap_checking_time_bound =
      collision_checking_params["overlap_checking_time_bound"].as<double>();

  // collision_checking_params
  YAML::Node waypoint_manager_params = config["waypoint_manager_param"];

  m_maptrack_size = waypoint_manager_params["maptrack_size"].as<int>();

  // minimal checking
  if (m_maptrack_size < 0)
  {
    throw std::runtime_error(
        "m_maptrack_size can not be less than zero. Check config file.");
  }

  if (m_config_planning_dt <= 0.0)
  {
    throw std::runtime_error(
        "m_config_planning_dt can not be less than zero. Check config file.");
  }

  if (m_config_planning_horizon < m_config_planning_dt)
  {
    throw std::runtime_error("m_config_planning_horizon can not be shorter "
                             "than m_config_planning_dt. Check config file.");
  }

  if (m_config_max_accel <= 0.0)
  {
    throw std::runtime_error(
        "m_config_max_accel can not be less than zero. Check config file.");
  }

  if (m_config_overtaking_longitudinal_margin <= 0.0 ||
      m_config_overtaking_lateral_margin <= 0.0 ||
      m_config_merging_longitudinal_margin <= 0.0)
  {
    throw std::runtime_error(
        "Safety margin can not be less than zero. Check config file.");
  }

  // if (m_config_overlap_checking_dist_bound <=
  //   nif::common::vehicle_param::VEH_WHEEL_BASE)
  // {
  //   throw std::runtime_error(
  //           "m_config_overlap_checking_dist_bound can not be "
  //           "less than Vehicle wheel base(4.921m). Check config file.");
  // }

  if (m_config_merge_allow_dist <= 0.0)
  {
    throw std::runtime_error("m_config_merge_allow_dist can not be "
                             "less than zero. Check config file.");
  }

  if (m_config_overlap_checking_dist_bound <= 0.0)
  {
    throw std::runtime_error("m_config_overlap_checking_dist_bound can not be "
                             "less than zero. Check config file.");
  }

  if (m_config_overlap_checking_time_bound <= 0.0)
  {
    throw std::runtime_error("m_config_overlap_checking_time_bound can not be "
                             "less than zero. Check config file.");
  }
}
// csv -> x, y vector
tuple<vector<double>, vector<double>>
DynamicPlannerNode::loadCSVfile(const std::string &wpt_file_path_)
{
  ifstream inputFile(wpt_file_path_);
  vector<double> vec_x, vec_y;

  while (inputFile)
  {
    string s;
    if (!getline(inputFile, s))
    {
      break;
    }
    if (s[0] != '#')
    {
      istringstream ss(s);
      int cnt = 0;
      bool nan_flg = false;
      while (ss)
      {
        string line;
        if (!getline(ss, line, ','))
        {
          break;
        }
        try
        {
          if (cnt == 0)
          {
            vec_x.push_back(stof(line));
          }
          else if (cnt == 1)
          {
            vec_y.push_back(stof(line));
          }
        }
        catch (const invalid_argument e)
        {
          cout << "NaN found in file " << wpt_file_path_ << endl;
          e.what();
          nan_flg = true;
        }
        cnt++;
      }
    }
  }

  if (!inputFile.eof())
  {
    cerr << "Could not read file " << wpt_file_path_ << "\n";
    __throw_invalid_argument("File not found.");
  }

  if (vec_x.size() == 0 || vec_y.size() == 0 ||
      (vec_x.size() != vec_y.size()))
  {
    __throw_invalid_argument("WPT SIZE ERROR.");
  }

  return std::make_tuple(vec_x, vec_y);
}
// not using the result
void DynamicPlannerNode::detectionResultCallback(
    const nif::common::msgs::PerceptionResultList::SharedPtr msg)
{
  // TODO: Detection result health check

  // TRACKING RESULT CALLBACK (GLOBAL COORDINATE)
  if (m_det_callback_first_run)
  {
    m_det_callback_first_run = false;
  }
  else
  {
    m_prev_det_global = m_cur_det_global;
  }

  if (!msg->perception_list.empty())
  {
    m_cur_det_global = msg->perception_list[0];
  }
}

void DynamicPlannerNode::predictionResultCallback(
    const nif_msgs::msg::DynamicTrajectory::SharedPtr msg)
{
  // TODO: Prediction result health check

  if (m_oppo_pred_callback_first_run)
  {
    m_oppo_pred_callback_first_run = false;
  }
  else
  {
    m_prev_oppo_pred_result = m_cur_oppo_pred_result;
  }
  m_prev_oppo_pred_last_update = this->now(); // TODO msg->header.stamp;
  m_cur_oppo_pred_result = *msg;
}
// from opponent prediction, opponent reference line info
void DynamicPlannerNode::opponentReferenceCallback(
    const nif_msgs::msg::OpponentRefline::SharedPtr msg)
{
  m_opponent_reference_line = std::move(*msg);
}
void DynamicPlannerNode::publishEmptyTrajectory()
{
  nif_msgs::msg::DynamicTrajectory empty_traj;
  nav_msgs::msg::Path empty_path;

  empty_traj.header.stamp = this->now();
  empty_path.header.stamp = this->now();

  empty_traj.header.frame_id = nif::common::frame_id::localization::BASE_LINK;
  m_ego_traj_body_pub->publish(empty_traj);

  empty_traj.header.frame_id = nif::common::frame_id::localization::ODOM;
  m_ego_traj_global_pub->publish(empty_traj);

  empty_path.header.frame_id = nif::common::frame_id::localization::BASE_LINK;
  m_ego_traj_body_vis_pub->publish(empty_path);

  empty_path.header.frame_id = nif::common::frame_id::localization::ODOM;
  m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
}

// void DynamicPlannerNode::publishPlannedTrajectory(bool vis_flg_) {
//   m_cur_ego_planned_result_body.trajectory_path.poses.clear();
//   m_cur_ego_planned_result_global.trajectory_path.poses.clear();

//   m_cur_ego_planned_result_body.header.stamp = this->now();
//   m_cur_ego_planned_result_body.header.frame_id =
//       nif::common::frame_id::localization::BASE_LINK;
//   m_cur_ego_planned_result_global.header.stamp = this->now();
//   m_cur_ego_planned_result_global.header.frame_id =
//       nif::common::frame_id::localization::ODOM;

//   m_cur_ego_planned_result_body.trajectory_type =
//       nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;
//   m_cur_ego_planned_result_global.trajectory_type =
//       nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;

//   // Current idx
//   m_ego_cur_idx_in_planned_traj = calcCurIdxFromDynamicTraj(m_cur_planned_traj);

//   int tmp_wpt_len = 100;

//   for (int wpt_idx = 0; wpt_idx < tmp_wpt_len; wpt_idx++) {
//     geometry_msgs::msg::PoseStamped ps_body;
//     geometry_msgs::msg::PoseStamped ps_global;

//     int target_idx_in_full_path = m_ego_cur_idx_in_planned_traj + wpt_idx;
//     // index wrapping
//     if (target_idx_in_full_path >=
//         m_cur_planned_traj.trajectory_path.poses.size()) {
//       target_idx_in_full_path -=
//           m_cur_planned_traj.trajectory_path.poses.size();
//     }

//     ps_global =
//         m_cur_planned_traj.trajectory_path.poses[target_idx_in_full_path];
//     ps_global.header.frame_id = nif::common::frame_id::localization::ODOM;

//     ps_body =
//         common::utils::coordination::getPtGlobaltoBody(m_ego_odom, ps_global);
//     ps_body.header.frame_id = nif::common::frame_id::localization::BASE_LINK;

//     m_cur_ego_planned_result_body.trajectory_path.poses.push_back(ps_body);
//     m_cur_ego_planned_result_global.trajectory_path.poses.push_back(ps_global);

//     // TODO : no idea to assign the velocity, time, progress.... FIX THIS NEAR
//     // SOON!!!
//   }
//   m_ego_traj_body_pub->publish(m_cur_ego_planned_result_body);
//   m_ego_traj_global_pub->publish(m_cur_ego_planned_result_global);

//   if (vis_flg_) {
//     m_ego_planned_vis_path_body = m_cur_ego_planned_result_body.trajectory_path;
//     m_ego_planned_vis_path_global =
//         m_cur_ego_planned_result_global.trajectory_path;

//     m_ego_planned_vis_path_body.header.frame_id =
//         nif::common::frame_id::localization::BASE_LINK;
//     m_ego_planned_vis_path_global.header.frame_id =
//         nif::common::frame_id::localization::ODOM;

//     m_ego_planned_vis_path_body.header.stamp = this->now();
//     m_ego_planned_vis_path_global.header.stamp = this->now();

//     m_ego_traj_body_vis_pub->publish(m_ego_planned_vis_path_body);
//     m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
//   }
// }

// double
// DynamicPlannerNode::getProgress(const geometry_msgs::msg::Pose &pt_global_,
//                                 pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
//   double progress;

//   std::vector<int> pointId_vector;
//   std::vector<float> pointRadius_vector;
//   pcl::PointXY *searchPoint = new pcl::PointXY();
//   searchPoint->x = pt_global_.position.x;
//   searchPoint->y = pt_global_.position.y;
//   int index = 0;

//   if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
//                                   pointRadius_vector) > 0) {
//     index = pointId_vector[0];
//   } else {
//     // TODO : what happens?
//   }

//   return index * m_config_spline_interval;
// }

// double
// DynamicPlannerNode::getProgress(const double &pt_x_, const double &pt_y_,
//                                 pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
//   double progress;

//   std::vector<int> pointId_vector;
//   std::vector<float> pointRadius_vector;
//   pcl::PointXY *searchPoint = new pcl::PointXY();
//   searchPoint->x = pt_x_;
//   searchPoint->y = pt_y_;
//   int index = 0;

//   if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
//                                   pointRadius_vector) > 0) {
//     index = pointId_vector[0];
//   } else {
//     // TODO : what happens?
//   }

//   return index * m_config_spline_interval;
// }

// double
// DynamicPlannerNode::getCurIdx(const double &pt_x_, const double &pt_y_,
//                               pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
//   double progress;

//   std::vector<int> pointId_vector;
//   std::vector<float> pointRadius_vector;
//   pcl::PointXY *searchPoint = new pcl::PointXY();
//   searchPoint->x = pt_x_;
//   searchPoint->y = pt_y_;
//   int index = 0;

//   if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
//                                   pointRadius_vector) > 0) {
//     index = pointId_vector[0];
//   } else {
//     // TODO : what happens?
//   }

//   return index;
// }

double DynamicPlannerNode::getCurIdx(const double &pt_x_, const double &pt_y_,
                                     const nav_msgs::msg::Path &target_path_)
{
  int closest_idx = 0;
  double min_dist = nif::common::constants::numeric::INF;
  for (int i = 0; i < target_path_.poses.size(); i++)
  {
    double dist = sqrt(pow(pt_x_ - target_path_.poses[i].pose.position.x, 2) +
                       pow(pt_y_ - target_path_.poses[i].pose.position.y, 2));
    if (dist < min_dist)
    {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

// double DynamicPlannerNode::calcCTE(const geometry_msgs::msg::Pose &pt_global_,
//                                    pcl::KdTreeFLANN<pcl::PointXY> &target_tree_,
//                                    pcl::PointCloud<pcl::PointXY>::Ptr &pc_) {
//   double cte = 0;
//   double progress;
//   int sign;

//   std::vector<int> pointId_vector;
//   std::vector<float> pointRadius_vector;
//   pcl::PointXY *searchPoint = new pcl::PointXY();
//   searchPoint->x = pt_global_.position.x;
//   searchPoint->y = pt_global_.position.y;

//   if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
//                                   pointRadius_vector) > 0) {
//     cte = pointRadius_vector[0];
//     progress = pointId_vector[0] * m_config_spline_interval;

//     int next_idx = pointId_vector[0] + 1;
//     double next_x, next_y;
//     if (pc_->points.size() < next_idx) {
//       next_idx = next_idx - pc_->points.size();
//     }
//     next_x = pc_->points[next_idx].x;
//     next_y = pc_->points[next_idx].y;

//     auto cross_product = next_x * pc_->points[pointId_vector[0]].y -
//                          next_y * pc_->points[pointId_vector[0]].x;
//     if (cross_product < 0) {
//       sign = -1;
//     } else {
//       sign = 1;
//     }

//     cte = cte * sign;

//   } else {
//     // TODO : what happens?
//   }

//   return cte;
// }

// pcl::PointCloud<pcl::PointXY>::Ptr
// DynamicPlannerNode::genPointCloudFromVec(vector<double> &x_,
//                                          vector<double> &y_) {
//   pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);

//   // Generate pointcloud data
//   cloud->width = x_.size();
//   cloud->height = 1;
//   cloud->points.resize(cloud->width * cloud->height);

//   for (std::size_t i = 0; i < cloud->size(); ++i) {
//     (*cloud)[i].x = x_[i];
//     (*cloud)[i].y = y_[i];
//   }
//   return cloud;
// }

// double DynamicPlannerNode::calcProgressDiff(
//     const geometry_msgs::msg::Pose &ego_pt_global_,
//     const geometry_msgs::msg::Pose &target_pt_global_,
//     pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
//   auto ego_progress = getProgress(ego_pt_global_, target_tree_);
//   auto target_progress = getProgress(target_pt_global_, target_tree_);

//   // TODO : progress wrapping is needed!!!!!!!!!!!!!!!!!!!
//   return ego_progress - target_progress;
// }

nav_msgs::msg::Path
DynamicPlannerNode::xyyawVec2Path(std::vector<double> &x_,
                                  std::vector<double> &y_,
                                  std::vector<double> &yaw_rad_)
{
  nav_msgs::msg::Path output;
  output.header.frame_id = nif::common::frame_id::localization::ODOM;

  for (int i = 0; i < x_.size(); i++)
  {
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.position.x = x_[i];
    pt.pose.position.y = y_[i];
    pt.pose.position.z = 0.0;
    pt.pose.orientation =
        nif::common::utils::coordination::euler2quat(yaw_rad_[i], 0.0, 0.0);
    output.poses.push_back(pt);
  }

  return output;
}

// tuple<double, double> DynamicPlannerNode::calcProgressNCTE(
//     const geometry_msgs::msg::Pose &pt_global_,
//     pcl::KdTreeFLANN<pcl::PointXY> &target_tree_,
//     pcl::PointCloud<pcl::PointXY>::Ptr &pc_) {
//   double cte = 0;
//   double progress;
//   int sign;

//   std::vector<int> pointId_vector;
//   std::vector<float> pointRadius_vector;
//   pcl::PointXY *searchPoint = new pcl::PointXY();
//   searchPoint->x = pt_global_.position.x;
//   searchPoint->y = pt_global_.position.y;

//   if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
//                                   pointRadius_vector) > 0) {
//     cte = pointRadius_vector[0];
//     progress = pointId_vector[0] * m_config_spline_interval;

//     int next_idx = pointId_vector[0] + 1;
//     double next_x, next_y;
//     if (pc_->points.size() < next_idx) {
//       next_idx = next_idx - pc_->points.size();
//     }
//     next_x = pc_->points[next_idx].x;
//     next_y = pc_->points[next_idx].y;

//     auto cross_product = next_x * pc_->points[pointId_vector[0]].y -
//                          next_y * pc_->points[pointId_vector[0]].x;
//     if (cross_product < 0) {
//       sign = -1;
//     } else {
//       sign = 1;
//     }

//     cte = cte * sign;

//   } else {
//     // TODO : what happens?
//   }

//   return std::make_tuple(progress, cte);
// }

tuple<double, double>
DynamicPlannerNode::calcProgressNCTE(const geometry_msgs::msg::Pose &pt_global_,
                                     nav_msgs::msg::Path &target_path_)
{
  double cte = 0;
  double progress;
  int sign;
  int closest_idx = 0;

  double min_dist = 1000000000;

  for (int i = 0; i < target_path_.poses.size(); i++)
  {
    // std::cout << "calcProgressNCTE" << std::endl;
    double dist = sqrt(
        pow(pt_global_.position.x - target_path_.poses[i].pose.position.x, 2) +
        pow(pt_global_.position.y - target_path_.poses[i].pose.position.y, 2));
    if (dist < min_dist)
    {
      min_dist = dist;
      closest_idx = i;
    }
  }

  int next_idx = closest_idx + 1;
  double next_x, next_y;
  double cur_x, cur_y;
  if (target_path_.poses.size() <= next_idx)
  {
    next_idx = next_idx - target_path_.poses.size();
  }

  cur_x = target_path_.poses[closest_idx].pose.position.x;
  cur_y = target_path_.poses[closest_idx].pose.position.y;

  next_x = target_path_.poses[next_idx].pose.position.x;
  next_y = target_path_.poses[next_idx].pose.position.y;

  // vector 1 : (next_x - pt_global_.position.x , next_y - pt_global_.position.y
  // , )
  // vector 2 : (cur_x - pt_global_.position.x , cur_y - pt_global_.position.y
  // , )

  auto cross_product =
      (next_x - pt_global_.position.x) * (cur_y - pt_global_.position.y) -
      (cur_x - pt_global_.position.x) * (next_y - pt_global_.position.y);
  if (cross_product < 0)
  {
    sign = 1;
  }
  else
  {
    sign = -1;
  }

  cte = min_dist * sign;
  progress = closest_idx * m_config_spline_interval; // global progress

  return std::make_tuple(progress, cte);
}

int DynamicPlannerNode::calcCurIdxFromDynamicTraj(
    const nif_msgs::msg::DynamicTrajectory &msg)
{
  int cur_idx = 0;
  double min_dist = 1000000000;

  for (int i = 0; i < msg.trajectory_path.poses.size(); i++)
  {
    double dist = sqrt(pow(m_ego_odom.pose.pose.position.x -
                               msg.trajectory_path.poses[i].pose.position.x,
                           2) +
                       pow(m_ego_odom.pose.pose.position.y -
                               msg.trajectory_path.poses[i].pose.position.y,
                           2));
    if (dist < min_dist)
    {
      min_dist = dist;
      cur_idx = i;
    }
  }
  return cur_idx;
}

// nif_msgs::msg::DynamicTrajectory DynamicPlannerNode::stitchFrenetToPath(
//     std::shared_ptr<FrenetPath> &frenet_segment_,
//     pcl::KdTreeFLANN<pcl::PointXY> &target_tree_,
//     nav_msgs::msg::Path &target_path_) {
//   nif_msgs::msg::DynamicTrajectory out;

//   // find closest index of target_path with respect to the start point of the
//   // frenet segment
//   auto vec_x = frenet_segment_->points_x();
//   auto vec_y = frenet_segment_->points_y();
//   auto vec_yaw = frenet_segment_->yaw();

//   auto cloest_pt_idx_wrt_segment_start_pt =
//       getCurIdx(vec_x[0], vec_y[0], target_tree_);
//   auto cloest_pt_idx_wrt_segment_end_pt =
//       getCurIdx(vec_x.back(), vec_y.back(), target_tree_);

//   for (int i = 0; i < vec_x.size(); i++) {
//     geometry_msgs::msg::PoseStamped ps;
//     ps.pose.position.x = vec_x[i];
//     ps.pose.position.y = vec_y[i];
//     ps.pose.orientation =
//         nif::common::utils::coordination::euler2quat(vec_yaw[i], 0.0, 0.0);

//     out.trajectory_path.poses.push_back(ps);
//   }

//   if (cloest_pt_idx_wrt_segment_start_pt > cloest_pt_idx_wrt_segment_end_pt) {
//     // index wrapping
//     out.trajectory_path.poses.insert(
//         out.trajectory_path.poses.end(),
//         target_path_.poses.begin() + cloest_pt_idx_wrt_segment_end_pt,
//         target_path_.poses.begin() + cloest_pt_idx_wrt_segment_start_pt);
//   } else {
//     out.trajectory_path.poses.insert(out.trajectory_path.poses.end(),
//                                      target_path_.poses.begin() +
//                                          cloest_pt_idx_wrt_segment_end_pt,
//                                      target_path_.poses.end());
//     out.trajectory_path.poses.insert(
//         out.trajectory_path.poses.end(), target_path_.poses.begin(),
//         target_path_.poses.begin() + cloest_pt_idx_wrt_segment_start_pt);
//   }
//   return out;
// }

nif_msgs::msg::DynamicTrajectory DynamicPlannerNode::stitchFrenetToPath(
    std::shared_ptr<FrenetPath> &frenet_segment_,
    nav_msgs::msg::Path &target_path_)
{
  nif_msgs::msg::DynamicTrajectory out;

  auto vec_x = frenet_segment_->points_x();
  auto vec_y = frenet_segment_->points_y();
  auto vec_yaw = frenet_segment_->yaw();

  // find closest index of target_path with respect to the start point of the
  // frenet segment
  auto cloest_pt_idx_wrt_segment_start_pt =
      getCurIdx(vec_x[0], vec_y[0], target_path_);
  // find closest index of target_path with respect to the end point of the
  // frenet segment + 2 (to cope with the case that the nearest point is behind
  // )
  auto cloest_pt_idx_wrt_segment_end_pt =
      getCurIdx(vec_x.back(), vec_y.back(), target_path_) + 2;

  m_reset_wpt_idx = cloest_pt_idx_wrt_segment_end_pt;

  // Index wrapping for end point of the frenet
  if (cloest_pt_idx_wrt_segment_end_pt >= target_path_.poses.size())
  {
    cloest_pt_idx_wrt_segment_end_pt -= target_path_.poses.size();
  }

  // Add frenet part first
  for (int i = 0; i < vec_x.size(); i++)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = vec_x[i];
    ps.pose.position.y = vec_y[i];
    ps.pose.orientation =
        nif::common::utils::coordination::euler2quat(vec_yaw[i], 0.0, 0.0);
    ps.header.frame_id = "odom";
    out.trajectory_path.poses.push_back(ps);
  }

  if (cloest_pt_idx_wrt_segment_start_pt > cloest_pt_idx_wrt_segment_end_pt)
  {
    // index wrapping (frenet start from end part of the target path.)
    out.trajectory_path.poses.insert(
        out.trajectory_path.poses.end(),
        target_path_.poses.begin() + cloest_pt_idx_wrt_segment_end_pt,
        target_path_.poses.begin() + cloest_pt_idx_wrt_segment_start_pt);
  }
  else
  {
    out.trajectory_path.poses.insert(out.trajectory_path.poses.end(),
                                     target_path_.poses.begin() +
                                         cloest_pt_idx_wrt_segment_end_pt,
                                     target_path_.poses.end());
    out.trajectory_path.poses.insert(
        out.trajectory_path.poses.end(), target_path_.poses.begin(),
        target_path_.poses.begin() + cloest_pt_idx_wrt_segment_start_pt);
  }

  return out;
}

double DynamicPlannerNode::getProgress(
    const geometry_msgs::msg::Pose &pt_global_,
    const nif_msgs::msg::DynamicTrajectory &target_traj)
{
  return getProgress(pt_global_.position.x, pt_global_.position.y, target_traj);
}

double DynamicPlannerNode::getProgress(
    const double &pt_x_, const double &pt_y_,
    const nif_msgs::msg::DynamicTrajectory &target_traj)
{
  double out;

  int closest_idx = 0;
  double min_dist = 1000000000;

  for (int i = 0; i < target_traj.trajectory_path.poses.size(); i++)
  {
    double dist = sqrt(
        pow(pt_x_ - target_traj.trajectory_path.poses[i].pose.position.x, 2) +
        pow(pt_y_ - target_traj.trajectory_path.poses[i].pose.position.y, 2));
    if (dist < min_dist)
    {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx * m_config_spline_interval;
}

// nav_msgs::msg::Path DynamicPlannerNode::getIntervalPath(
//     const geometry_msgs::msg::Pose &start_global_,
//     const geometry_msgs::msg::Pose &end_global_,
//     const nif_msgs::msg::DynamicTrajectory &target_traj) {
//   // Inside here, progress wrapping is done.

//   return getIntervalPath(start_global_.position.x, start_global_.position.y,
//                          end_global_.position.x, end_global_.position.y,
//                          target_traj);
// }

// nav_msgs::msg::Path DynamicPlannerNode::getIntervalPath(
//     const double &start_x_, const double &start_y_, const double &end_x_,
//     const double &end_y_, const nif_msgs::msg::DynamicTrajectory &target_traj) {
//   // Inside here, progress wrapping is done.

//   nav_msgs::msg::Path interval_path_out;

//   auto start_pt_progress = getProgress(start_x_, start_y_, target_traj);
//   auto end_pt_progress = getProgress(end_x_, end_y_, target_traj);
//   auto start_pt_idx = int(start_pt_progress / m_config_spline_interval);
//   auto end_pt_idx = int(end_pt_progress / m_config_spline_interval);

//   ////////////////////
//   // PROGRESS WRAPPING
//   ////////////////////

//   if (end_pt_idx - start_pt_progress > 0) {
//     interval_path_out.poses = std::vector<geometry_msgs::msg::PoseStamped>(
//         target_traj.trajectory_path.poses.begin() + start_pt_idx,
//         target_traj.trajectory_path.poses.begin() + end_pt_idx);
//   } else {
//     interval_path_out.poses = std::vector<geometry_msgs::msg::PoseStamped>(
//         target_traj.trajectory_path.poses.begin() + start_pt_idx,
//         target_traj.trajectory_path.poses.end());

//     interval_path_out.poses.insert(interval_path_out.poses.end(),
//                                    target_traj.trajectory_path.poses.begin(),
//                                    target_traj.trajectory_path.poses.begin() +
//                                        end_pt_idx);
//   }

//   return interval_path_out;
// }

nav_msgs::msg::Path DynamicPlannerNode::getCertainLenOfPathSeg(
    const double &start_x_, const double &start_y_,
    const nav_msgs::msg::Path &target_path_, const int &idx_length)
{
  if (target_path_.poses.empty())
  {
    RCLCPP_ERROR_ONCE(this->get_logger(),
                      "In side of getCertainLenOfPathSeg : target path is "
                      "empty. Return empty path.");
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = nif::common::frame_id::localization::ODOM;
    return empty_path;
  }

  nav_msgs::msg::Path out;
  out.header.frame_id = nif::common::frame_id::localization::ODOM;
  out.poses.resize(
      std::min(static_cast<unsigned int>(idx_length),
               static_cast<unsigned int>(target_path_.poses.size())));

  auto cur_idx_on_target_path = getCurIdx(start_x_, start_y_, target_path_);

  for (int idx = 0; idx < out.poses.size(); idx++)
  {
    // index wrapping
    int idx_on_target_path = cur_idx_on_target_path + idx;
    if (idx_on_target_path >= target_path_.poses.size())
    {
      idx_on_target_path -= target_path_.poses.size();
    }
    out.poses[idx] = target_path_.poses[idx_on_target_path];
  }
  // std::cout << out.poses.size() << std::endl;
  return out;
}

// void DynamicPlannerNode::publishPlannedTrajectory(
//     nif_msgs::msg::DynamicTrajectory &traj_, bool is_acc_, bool vis_) {
//   traj_.header.frame_id = nif::common::frame_id::localization::ODOM;
//   traj_.trajectory_type = traj_.TRAJECTORY_TYPE_PLANNING;

//   if (is_acc_) {
//     traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_FOLLOW;
//   } else {
//     traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_STRAIGHT;
//   }

//   traj_.trajectory_path.header.frame_id = traj_.header.frame_id;
//   m_ego_traj_global_pub->publish(traj_);

//   if (vis_) {
//     m_ego_planned_vis_path_global = traj_.trajectory_path;
//     m_ego_planned_vis_path_global.header.frame_id =
//         nif::common::frame_id::localization::ODOM;
//     m_ego_planned_vis_path_global.header.stamp = this->now();
//     m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
//   }
// }

void DynamicPlannerNode::publishPlannedTrajectory(
    nif_msgs::msg::DynamicTrajectory &traj_, int32_t longi_type_,
    int32_t lat_type_, bool vis_)
{
  traj_.header.frame_id = nif::common::frame_id::localization::ODOM;
  traj_.trajectory_type = traj_.TRAJECTORY_TYPE_PLANNING;
  // std::cout << "dynamic trajectory length: " <<traj_.trajectory_path.poses.size() << std::endl;
  if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::STRAIGHT)
  {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_STRAIGHT;
  }
  else if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::FOLLOW)
  {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_FOLLOW;
  }
  else if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::ESTOP)
  {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_ESTOP;
  }
  else
  {
    // std::cout << "Unknown type of longitudinal control type." << std::endl;
  }

  if (lat_type_ == LATERAL_PLANNING_TYPE::KEEP)
  {
    traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_KEEP;
  }
  else if (lat_type_ == LATERAL_PLANNING_TYPE::MERGE)
  {
    traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_MERGE;
  }
  else if (lat_type_ == LATERAL_PLANNING_TYPE::CHANGE_PATH)
  {
    traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_CHANGE_PATH;
  }
  else
  {
    // std::cout << "Unknown type of lateral control type." << std::endl;
  }

  traj_.trajectory_path.header.frame_id = traj_.header.frame_id;
  m_ego_traj_global_pub->publish(traj_);

  if (vis_)
  {
    m_ego_planned_vis_path_global = traj_.trajectory_path;
    m_ego_planned_vis_path_global.header.frame_id =
        nif::common::frame_id::localization::ODOM;
    m_ego_planned_vis_path_global.header.stamp = this->now();
    m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
  }

  auto d_first = get<1>(calcProgressNCTE(traj_.trajectory_path.poses.front().pose, m_center_path));
  auto d_last = get<1>(calcProgressNCTE(traj_.trajectory_path.poses.back().pose, m_center_path));
  bool is_path_cross_center = (d_first * d_last) < 0;
  int size = traj_.trajectory_path.poses.size();
  // std::cout << d_first << ", " << d_last <<  ", " << is_path_cross_center << ", " << size << std::endl;
}

void DynamicPlannerNode::checkSwitchToStaticWPT(int cur_wpt_idx_)
{

  if (cur_wpt_idx_ >= m_reset_wpt_idx &&
      m_reset_wpt_idx != RESET_PATH_TYPE::NONE &&
      m_reset_target_path_idx != RESET_PATH_TYPE::NONE)
  {
    if (m_reset_target_path_idx == RESET_PATH_TYPE::RACE_LINE)
    {
      // target path is the racing line
      m_cur_planned_traj.trajectory_path = m_racingline_path;
      m_reset_wpt_idx = RESET_PATH_TYPE::NONE;
      m_reset_target_path_idx = RESET_PATH_TYPE::NONE;
    }
    else if (m_reset_target_path_idx == RESET_PATH_TYPE::DEFENDER_LINE)
    {
      // target path is the racing line
      m_cur_planned_traj.trajectory_path = m_defenderline_path;
      m_reset_wpt_idx = RESET_PATH_TYPE::NONE;
      m_reset_target_path_idx = RESET_PATH_TYPE::NONE;
    }
    else if (m_reset_target_path_idx == RESET_PATH_TYPE::STAY_BEHIND)
    {
      // target path is the racing line
      m_cur_planned_traj.trajectory_path = m_staybehind_path;
      m_reset_wpt_idx = RESET_PATH_TYPE::NONE;
      m_reset_target_path_idx = RESET_PATH_TYPE::NONE;
    }
    else
    {
      // target path is one of the overtaking path candidates
      m_cur_planned_traj.trajectory_path =
          m_overtaking_candidates_path_vec[m_reset_target_path_idx];
      m_reset_wpt_idx = RESET_PATH_TYPE::NONE;
      m_reset_target_path_idx = RESET_PATH_TYPE::NONE;
    }
  }
}

// void DynamicPlannerNode::publishPlannedTrajectory(
//     nif_msgs::msg::DynamicTrajectory &traj_, int longi_type_, int lat_type_,
//     int target_path_, bool vis_) {
//   traj_.header.frame_id = nif::common::frame_id::localization::ODOM;
//   traj_.trajectory_type = traj_.TRAJECTORY_TYPE_PLANNING;

//   if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::STRAIGHT) {
//     traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_STRAIGHT;
//   } else if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::FOLLOW) {
//     traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_FOLLOW;
//   } else if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::ESTOP) {
//     traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_ESTOP;
//   } else {
//     // std::cout << "Unknown type of longitudinal control type." << std::endl;
//   }

//   if (lat_type_ == LATERAL_PLANNING_TYPE::KEEP) {
//     traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_KEEP;
//   } else if (lat_type_ == LATERAL_PLANNING_TYPE::MERGE) {
//     traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_MERGE;
//   } else if (lat_type_ == LATERAL_PLANNING_TYPE::CHANGE_PATH) {
//     traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_CHANGE_PATH;
//   } else {
//     // std::cout << "Unknown type of lateral control type." << std::endl;
//   }

//   traj_.trajectory_path.header.frame_id = traj_.header.frame_id;
//   m_ego_traj_global_pub->publish(traj_);

//   if (vis_) {
//     m_ego_planned_vis_path_global = traj_.trajectory_path;
//     m_ego_planned_vis_path_global.header.frame_id =
//         nif::common::frame_id::localization::ODOM;
//     m_ego_planned_vis_path_global.header.stamp = this->now();
//     m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
//   }
// }

nif_msgs::msg::DynamicTrajectory
DynamicPlannerNode::findOvertakingPath(double default_planning_time_min, double allowable_maximum_vy, bool for_overtake)
{
  // std::cout << "debug4-1" << std::endl;
  vector<std::shared_ptr<FrenetPath>> collision_free_frenet_vec;
  vector<double> collision_free_frenet_progress_vec;
  vector<int> collision_free_frenet_index_vec;
  bool can_overtake = false;
  nav_msgs::msg::Path cur_path_seg;
  nif_msgs::msg::DynamicTrajectory cur_traj;
  // Generate the frenet candidates to all overtaking path candidates
  // Right side first
  for (int path_candidate_idx = 0;
       path_candidate_idx < m_overtaking_candidates_path_vec.size();
       path_candidate_idx++)
  {

    if (m_overtaking_candidates_path_vec[path_candidate_idx]
            .poses.empty() ||
        can_overtake)
    {
      continue;
    }

    auto progressNcte = calcProgressNCTE(
        m_ego_odom.pose.pose,
        m_overtaking_candidates_path_vec[path_candidate_idx]);

    std::vector<std::shared_ptr<FrenetPath>>
        frenet_path_generation_result =
            m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                get<1>(progressNcte), // current_position_d
                get<0>(progressNcte), // current_position_s
                0.0,                  // current_velocity_d
                std::max(m_ego_odom.twist.twist.linear.x,
                         MIN_SPEED_MPS), // current_velocity_s
                0.0,                     // current_acceleration_d
                m_overtaking_candidates_spline_model_vec
                    [path_candidate_idx], // cubicSplineModel
                std::max(
                    abs(get<1>(progressNcte) / allowable_maximum_vy),
                    default_planning_time_min),
                std::max(
                    abs(get<1>(progressNcte) / allowable_maximum_vy),
                    default_planning_time_min) +
                    2.0 + 0.01,
                SAMPLING_TIME, 0.0, 0.0001, 0.1);

    if (frenet_path_generation_result.empty())
    {
      continue;
    }

    for (int frenet_idx = frenet_path_generation_result.size() - 1;
         frenet_idx >= 0; frenet_idx--)
    {
      //  Check collision in order of length of path (which means less
      //  jerky)
      auto frenet_candidate = frenet_path_generation_result[frenet_idx];

      auto stitched_path = stitchFrenetToPath(
          frenet_candidate,
          m_overtaking_candidates_path_vec[path_candidate_idx]);

      cur_path_seg =
          getCertainLenOfPathSeg(m_ego_odom.pose.pose.position.x,
                                 m_ego_odom.pose.pose.position.y,
                                 stitched_path.trajectory_path, 100);

      cur_traj =
          m_velocity_profiler_obj.velProfileWCollisionChecking(
              m_ego_odom, cur_path_seg, m_cur_oppo_pred_result,
              m_config_overlap_checking_dist_bound,
              m_config_overlap_checking_time_bound, false,
              1.0 /*, for_overtake*/); // @DEBUG

      if (!cur_traj.has_collision)
      {

        m_cur_planned_traj = stitched_path;
        m_reset_wpt_idx = getCurIdx(frenet_candidate->points_x().back(),
                                    frenet_candidate->points_y().back(),
                                    m_cur_planned_traj.trajectory_path);

        m_reset_target_path_idx = path_candidate_idx;
        m_last_update_target_path_alias =
            m_overtaking_candidates_file_path_vec[path_candidate_idx];
        m_last_lat_planning_type = LATERAL_PLANNING_TYPE::CHANGE_PATH;
        m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
        can_overtake = true;
        collision_free_frenet_vec.push_back(frenet_candidate);
        return cur_traj;
      }
    }
  }
  if (collision_free_frenet_vec.empty())
  {
    /////////////////////////////////////////
    // All path are cancled due to collisions
    /////////////////////////////////////////
    // std::cout << "debug4-2" << std::endl;

    cur_path_seg =
        getCertainLenOfPathSeg(m_ego_odom.pose.pose.position.x,
                               m_ego_odom.pose.pose.position.y,
                               m_cur_planned_traj.trajectory_path, 100);

    cur_traj = m_velocity_profiler_obj.velProfileForAcc(
        m_ego_odom, m_cur_oppo_pred_result,
        m_cur_det_global.obj_velocity_in_global.linear.x, cur_path_seg,
        1.0);

    // Publish cur_traj
    // Keep previous plan and do ACC
    m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
    m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
  }
  return cur_traj;
}
void DynamicPlannerNode::timer_callback_rule()
{
    // std::cout << "timer_callback_rule" << std::endl;

  rclcpp::Time temp_time = this->now();
  // std::cout << m_last_update_target_path_alias << std::endl;
  // ----------------------------------------------------------------
  // --------------------- SYSTEM health check ---------------------
  // ----------------------------------------------------------------
  auto &mission_status = this->getSystemStatus().mission_status;
  double mission_max_vel = mission_status.max_velocity_mps;
  auto planner_status_msg = nif_msgs::msg::PlannerStatus();
  planner_status_msg.stamp = this->now();
  // Set the maximum accel and decel following the mission manager
  m_mission_accel_max = abs(mission_status.zone_status.long_acceleration_max);
  m_mission_decel_max = abs(
      mission_status.zone_status.long_acceleration_min); // should be negative

  // Set to velocity profiler
  auto flg = m_velocity_profiler_obj.setConstraintMaxVel(mission_max_vel);
  auto success_flg =
      m_velocity_profiler_obj.setConstraintMaxAccel(m_mission_accel_max);
  success_flg =
      m_velocity_profiler_obj.setConstraintMaxDeccel(m_mission_decel_max);

  double acc_min_dist_straight = 0.0;
  double acc_time_headway_straight = 0.0;

  auto default_planning_time_max = 0.0;
  auto default_planning_time_min = 0.0;

  if (mission_status.zone_status.zone_type ==
      mission_status.zone_status.ZONE_TYPE_STRAIGHT)
  {
    // acc_min_dist_straight = ;
    // acc_time_headway_straight = ;
    // m_velocity_profiler_obj.setACCMindist(acc_min_dist_straight);
    // m_velocity_profiler_obj.setACCTimeHeadway(acc_time_headway_straight);

    // set planning horizon based on zone type and id
    default_planning_time_max = SEC_7;
    default_planning_time_min = SEC_2;
  }
  else
  {
    // acc_min_dist_straight = ;
    // acc_time_headway_straight = ;
    // m_velocity_profiler_obj.setACCMindist(acc_min_dist_straight);
    // m_velocity_profiler_obj.setACCTimeHeadway(acc_time_headway_straight);

    // set planning horizon based on zone type and id
    default_planning_time_max = SEC_5;
    default_planning_time_min = SEC_2;
  }
  // ---------------------------------------------------------------

  auto allowable_maximum_vy =
      abs(tan(m_acceptable_slip_angle_rad)) *
      std::max(m_ego_odom.twist.twist.linear.x, MIN_SPEED_MPS);
  allowable_maximum_vy = std::max(1.5, allowable_maximum_vy); // mps

  // ---------------------------------------------------------------
  // --------------- Oppo prediction health check ---------------
  // ---------------------------------------------------------------
  if (!m_oppo_pred_callback_first_run)
  {
    if (this->now() - m_prev_oppo_pred_last_update > rclcpp::Duration(2, 0))
    {
      m_cur_oppo_pred_result.trajectory_path.poses.clear();
      m_cur_oppo_pred_result.trajectory_velocity.clear();
      m_cur_oppo_pred_result.trajectory_timestamp_array.clear();
      m_cur_oppo_pred_result.trajectory_global_progress.clear();
    }
  }

  if (this->hasEgoOdometry())
  {
    // System ok
    nif::common::NodeStatusCode node_status = nif::common::NODE_OK;
    this->setNodeStatus(node_status);

    // Update ego odometry
    m_ego_odom = this->getEgoOdometry();

    // ---------------------------------------------------------------
    // USE PIT PATH
    // ---------------------------------------------------------------
    if (this->missionIs(nif::common::MissionStatus::MISSION_PIT_IN) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_SLOW) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_STANDBY) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_OUT) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_TO_TRACK) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_INIT) ||
        this->missionIs(nif::common::MissionStatus::MISSION_DEFAULT))
    {

      // std::cout << "PIT_mode!!!!!!!!!!" << std::endl;
      m_defender_mode_first_callback = true;
      m_race_mode_first_callback = true;
      m_stop_mode_first_callback = true;
      m_non_overtaking_mode_first_callback = true;

      // m_cur_planned_traj.trajectory_path.poses.clear();

      if (m_pit_mode_first_callback == true ||
          m_cur_planned_traj.trajectory_path.poses.empty() || this->missionIs(nif::common::MissionStatus::MISSION_PIT_INIT))
      {

        m_pit_mode_first_callback = false;
        // Switch to the pit line
        auto progressNCTE_pitline =
            calcProgressNCTE(m_ego_odom.pose.pose, m_pitline_path);
        // Merging frenet segment generation
        // Generate SINGLE frenet path segment

        double frenet_time;

        if (mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_PIT || abs(get<1>(progressNCTE_pitline)) < m_dist_same_lane_path_change)
        {
          frenet_time = std::max(abs(get<1>(progressNCTE_pitline) /
                                     allowable_maximum_vy),
                                 default_planning_time_min);
        }
        // else if (this->missionIs(nif::common::MissionStatus::MISSION_PIT_IN))
        //   frenet_time = 2 * std::max(abs(get<1>(progressNCTE_pitline) /
        //                           allowable_maximum_vy),
        //                       default_planning_time_max);
        else
        {
          frenet_time = std::max(abs(get<1>(progressNCTE_pitline) /
                                     allowable_maximum_vy),
                                 default_planning_time_max);
        }

        std::vector<std::shared_ptr<FrenetPath>>
            frenet_path_generation_result =
                m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                    get<1>(progressNCTE_pitline), // current_position_d
                    get<0>(progressNCTE_pitline), // current_position_s
                    0.0,                          // current_velocity_d
                    std::max(m_ego_odom.twist.twist.linear.x,
                             MIN_SPEED_MPS),       // current_velocity_s
                    0.0,                           // current_acceleration_d
                    get<4>(m_pitline_spline_data), // cubicSplineModel
                    frenet_time,
                    frenet_time + 0.01,
                    SAMPLING_TIME, 0.0, 0.0001, 0.1);
        if (frenet_path_generation_result.empty() ||
            frenet_path_generation_result[0]->points_x().empty())
        {
          // Abnormal situation
          // publish empty & Estop trajectory
          // @DEBUG:
          RCLCPP_ERROR_ONCE(this->get_logger(),
                            "CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                            "generation result or point vector is empty");
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
          nif_msgs::msg::DynamicTrajectory empty_traj;
          empty_traj.header.frame_id =
              nif::common::frame_id::localization::ODOM;
          publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                   m_last_lat_planning_type, true);
          return;
        }
        m_cur_planned_traj = stitchFrenetToPath(frenet_path_generation_result[0], m_pitline_path);

        m_last_update_target_path_alias = m_pitline_file_path;
      }
      auto cur_path_seg = getCertainLenOfPathSeg(
          m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
          m_cur_planned_traj.trajectory_path, 100);

      // Convert maptrack to trajectory and publish (only global / without
      // ACC)
      auto cur_traj = m_velocity_profiler_obj.velProfile(
          m_ego_odom, cur_path_seg, 1.0);

      m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
      m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
      publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                               m_last_lat_planning_type, true);

      planner_status_msg.planning_code = planner_status_msg.STRAIGHT;
      m_planner_status_pub->publish(planner_status_msg);
      // return;
    }

    //------------------------------------------------------------------------------------------------
    // USE RACELINE
    //------------------------------------------------------------------------------------------------
    else if (mission_status.mission_status_code ==
             nif::common::MissionStatus::MISSION_RACE)
    {

      m_pit_mode_first_callback = true;
      m_defender_mode_first_callback = true;
      m_stop_mode_first_callback = true;
      m_non_overtaking_mode_first_callback = true;

      // Change race mode reference path based on oppo prediction data
      // If oppo use racing line -> ego race on left race line
      // If oppo use left lane only -> ego race on right race line
      // No information -> racing line
      int race_mode_reset_path = RESET_PATH_TYPE::RACE_LINE;

      // if ((m_cur_planned_traj.trajectory_path.poses.empty()
      //   || m_race_mode_first_callback == true)
      //   && m_num_overtaking_candidates >= 2){
      //     if (m_opponent_reference_line.oppo_refline == m_opponent_reference_line.NO_INFORMATION){
      //       m_race_mode_file_path = m_racingline_file_path;
      //       m_race_mode_path = m_racingline_path;
      //       m_race_mode_spline_data = m_racingline_spline_data;
      //     }
      //     else if (m_opponent_reference_line.oppo_refline == m_opponent_reference_line.LEFT_RACE){
      //       m_race_mode_file_path = m_overtaking_candidates_file_path_vec[0];
      //       m_race_mode_path = m_overtaking_candidates_path_vec[0];
      //       m_race_mode_spline_data = m_overtaking_candidates_spline_data_vec[0];
      //       race_mode_reset_path = 0;
      //     }
      //     else if (m_opponent_reference_line.oppo_refline == m_opponent_reference_line.RACE){
      //       m_race_mode_file_path = m_overtaking_candidates_file_path_vec[1];
      //       m_race_mode_path = m_overtaking_candidates_path_vec[1];
      //       m_race_mode_spline_data = m_overtaking_candidates_spline_data_vec[1];
      //       race_mode_reset_path = 1;
      //     }
      //   }

      if (m_last_update_target_path_alias == m_race_mode_file_path)
        m_race_mode_first_callback = false;

      if (m_cur_planned_traj.trajectory_path.poses.empty() || (m_race_mode_first_callback == true && mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_STRAIGHT) // Because stay_behind -> attacker
      )
      {
        m_race_mode_first_callback = false;
        m_overtake_decision_counter = 0;

        auto progressNCTE_racingline =
            calcProgressNCTE(m_ego_odom.pose.pose, m_race_mode_path);

        // Merging frenet segment generation
        // Generate single frenet path segment
        double frenet_time = 0;

        if (abs(get<1>(progressNCTE_racingline)) < m_dist_same_lane_path_change)
        {
          frenet_time = std::max(abs(get<1>(progressNCTE_racingline) /
                                     allowable_maximum_vy),
                                 default_planning_time_min);
        }
        else
        {
          frenet_time = std::max(abs(get<1>(progressNCTE_racingline) /
                                     allowable_maximum_vy),
                                 default_planning_time_max);
        }
        std::vector<std::shared_ptr<FrenetPath>>
            frenet_path_generation_result =
                m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                    get<1>(progressNCTE_racingline), // current_position_d
                    get<0>(progressNCTE_racingline), // current_position_s
                    0.0,                             // current_velocity_d
                    std::max(m_ego_odom.twist.twist.linear.x,
                             MIN_SPEED_MPS),         // current_velocity_s
                    0.0,                             // current_acceleration_d
                    get<4>(m_race_mode_spline_data), // cubicSplineModel
                    frenet_time,
                    frenet_time + 0.01,
                    SAMPLING_TIME, 0.0, 0.0001, 0.1);

        if (frenet_path_generation_result.empty() ||
            frenet_path_generation_result[0]->points_x().empty())
        {
          // Abnormal situation
          // publish empty & Estop trajectory
          RCLCPP_ERROR_ONCE(
              this->get_logger(),
              "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n Frenet path "
              "generation result or point vector is empty");
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
          nif_msgs::msg::DynamicTrajectory empty_traj;
          empty_traj.header.frame_id =
              nif::common::frame_id::localization::ODOM;
          publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                   m_last_lat_planning_type, true);
          return;
        }

        m_cur_planned_traj = stitchFrenetToPath(
            frenet_path_generation_result[0], m_race_mode_path);

        m_reset_wpt_idx =
            getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                      frenet_path_generation_result[0]->points_y().back(),
                      m_cur_planned_traj.trajectory_path);

        // Set the target path index to -1 which means the racing line
        m_last_update_target_path_alias = m_race_mode_file_path;
        // m_reset_target_path_idx = RESET_PATH_TYPE::RACE_LINE;
        m_reset_target_path_idx = race_mode_reset_path;
      }

      ///////////////////////////////////////////
      // Change the target path to the static wpt
      ///////////////////////////////////////////
      auto cur_idx_on_previous_path = getCurIdx(
          m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
          m_cur_planned_traj.trajectory_path);

      checkSwitchToStaticWPT(cur_idx_on_previous_path);

      ///////////////////////////////////////////////////
      // Velocity profiling with the previous planned path
      ///////////////////////////////////////////////////
      auto cur_path_seg = getCertainLenOfPathSeg(
          m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
          m_cur_planned_traj.trajectory_path, 100);

      auto cur_traj = m_velocity_profiler_obj.velProfileWCollisionChecking(
          m_ego_odom, cur_path_seg, m_cur_oppo_pred_result,
          m_config_overlap_checking_dist_bound,
          m_config_overlap_checking_time_bound, false, 1.0 /*, false*/); // @DEBUG
      // std::cout << cur_path_seg.poses.size() << ", " << cur_traj.trajectory_path.poses.size() << std::endl;

      bool overtake_start = false;
      auto naive_gap = nif::common::constants::numeric::INF;
      if (!m_cur_oppo_pred_result.trajectory_path.poses.empty())
      {
        naive_gap = nif::common::utils::geometry::calEuclideanDistance(
            m_ego_odom.pose.pose,
            m_cur_oppo_pred_result.trajectory_path.poses.front().pose);
        geometry_msgs::msg::PoseStamped oppo_pose_in_body = nif::common::utils::coordination::getPtGlobaltoBody(this->getEgoOdometry(),
                                                                                                                m_cur_oppo_pred_result.trajectory_path.poses.front().pose);
        overtake_start = oppo_pose_in_body.pose.position.x < 5;
      }
      switch (m_overtake_status)
      {

      case OVERTAKE_STATUS::BEFORE:
        ///////////////////////////////////////
        // NO OVERTAKE ATTEMPT BEFORE MIDDLE OF THE FRONT STRETCH
        // Go back to race mode reference line for overtake attempt
        ///////////////////////////////////////
        if (/*mission_status.zone_status.zone_type != mission_status.zone_status.ZONE_TYPE_CORNER_MID
          && mission_status.zone_status.zone_type != mission_status.zone_status.ZONE_TYPE_CORNER_EXIT
          && */
            (mission_status.zone_status.zone_id > this->m_backstretch_zone[1]))
        {
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::CHANGE_PATH;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
          m_overtake_decision_counter = 0;
          m_race_mode_first_callback = true;
          // std::cout << "BEFORE, follow" << std::endl;
        }
        ///////////////////////////////////////
        // Too close to oppo, or already start passing
        // consider as attempt
        ///////////////////////////////////////
        else if (naive_gap < 10 || (overtake_start && naive_gap < 30))
        {
          //
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
          m_overtake_status = OVERTAKE_STATUS::ATTEMPT;
          m_overtake_decision_counter = 0;
        }
        ///////////////////////////////////////
        // Keep current planned traj
        // not considering the ACC in this case
        ///////////////////////////////////////
        else if (!cur_traj.has_collision)
        {
          // std::cout << "BEFORE, no collision" << std::endl;
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
        }
        ///////////////////////////////////////
        // Current planned traj has collision
        // Corner or second half of front stretch just follow
        ///////////////////////////////////////
        else if (mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_CORNER_MID || mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_CORNER_ENTRY || mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_CORNER_EXIT || mission_status.zone_status.zone_id < this->m_backstretch_zone[0])
        {
          // std::cout << "BEFORE, corner" << std::endl;
          m_overtake_decision_counter = 0;
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
        }
        /////////////////////////////////////////
        // In straight, collision detected but delayed decision
        // Do ACC
        /////////////////////////////////////////
        else if (m_overtake_decision_counter <= 10)
        { // = 0.2 sec (50Hz)
          // std::cout << "debug3" << std::endl;
          if (naive_gap < 2 * m_small_gap)
            m_overtake_decision_counter++;
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
        }
        ////////////////////////////////////////////////////////
        // ----------- Previous path has a collision -----------
        // -------- Search for the collision-free path ---------
        ////////////////////////////////////////////////////////
        else
        {
          // std::cout << "BEFORE, find overtake path" << std::endl;
          cur_traj = findOvertakingPath(default_planning_time_min, allowable_maximum_vy, true);
          m_overtake_status = OVERTAKE_STATUS::ATTEMPT;
          m_overtake_decision_counter = 0;
        }

        break;

      case OVERTAKE_STATUS::ATTEMPT:
      {
        auto d_first = get<1>(calcProgressNCTE(cur_path_seg.poses.front().pose, m_center_path));
        auto d_last = get<1>(calcProgressNCTE(cur_path_seg.poses.back().pose, m_center_path));
        double d_oppo = -d_first;
        bool is_path_cross_center = (d_first * d_last) < 0;
        if (!m_cur_oppo_pred_result.trajectory_path.poses.empty())
        {
          d_oppo = get<1>(calcProgressNCTE(m_cur_oppo_pred_result.trajectory_path.poses.front().pose, m_center_path));
        }
        bool oppo_in_same_lane = (d_first * d_oppo) > 0;

        // m_overtake_decision_counter = 0;
        /////////////////////////////////////
        // STATE CHANGE CONDITIONS
        /////////////////////////////////////
        if (naive_gap > 3 * m_small_gap && !overtake_start)
        {
          m_overtake_status = OVERTAKE_STATUS::BEFORE;
          m_overtake_decision_counter = 0;
          m_race_mode_first_callback = true;
        }
        else if (naive_gap > 20 && overtake_start && mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_STRAIGHT)
        {
          m_overtake_status = OVERTAKE_STATUS::DONE;
          m_overtake_decision_counter = 0;
        }
        /////////////////////////////
        // no collision
        /////////////////////////////
        // RIGHT OF WAY
        else if (is_path_cross_center
                 // || mission_status.zone_status.zone_type != mission_status.zone_status.ZONE_TYPE_CORNER_ENTRY)
                 && !oppo_in_same_lane && (cur_traj.has_collision || naive_gap < 20.0))
        {

          // std::cout << "RIGHT OF WAY!!!!!!!!!" << std::endl;
          if (m_overtake_decision_counter <= 5)
          {
            m_overtake_decision_counter++;
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
          }
          else
          {
            cur_traj = findOvertakingPath(default_planning_time_min, allowable_maximum_vy, false);
            m_overtake_status = OVERTAKE_STATUS::PASSING;
            m_overtake_decision_counter = 0;
          }
        }
        else if (!cur_traj.has_collision)
        {
          // std::cout << "ATTEMPT, no collision" << std::endl;
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
          if (naive_gap < 15 && !oppo_in_same_lane && !is_path_cross_center)
          {
            m_overtake_status = OVERTAKE_STATUS::PASSING;
            m_overtake_decision_counter = 0;
          }
        }
        // Collision case
        else
        {
          // std::cout << "is_path_cross_center: " << is_path_cross_center <<std::endl;
          // front stretch
          // else if (mission_status.zone_status.zone_type != mission_status.zone_status.ZONE_TYPE_CORNER_MID
          // && (mission_status.zone_status.zone_id < this->m_backstretch_zone[0] || mission_status.zone_status.zone_id > this->m_backstretch_zone[1])) {
          //   // std::cout << "debug7" << std::endl;
          //   m_overtake_decision_counter ++;
          //   m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          //   m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
          // }
          // back stretch collision case
          if (mission_status.zone_status.zone_type != mission_status.zone_status.ZONE_TYPE_CORNER_MID && mission_status.zone_status.zone_id >= this->m_backstretch_zone[0] && mission_status.zone_status.zone_id < this->m_backstretch_zone[1])
          {
            // std::cout << "ATTEMPT, waiting for reset" << std::endl;
            if (m_overtake_decision_counter < 30)
            { //=0.6 sec @ 50Hz
              m_overtake_decision_counter++;
              m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
              m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
            }
            else
            {
              m_overtake_decision_counter = 0;
              m_overtake_status = OVERTAKE_STATUS::BEFORE;
              m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
              m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
              // m_race_mode_first_callback = true;
            }
          }
          // front and corner
          else
          {
            // ONLY reset to racing line @ CORNER MID
            // std::cout << "ATTEMPT, follow" << std::endl;
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
          }
        }
        break;
      }
      case OVERTAKE_STATUS::PASSING:
      {
        // std::cout << "PASSING" << std::endl;
        auto d_first = get<1>(calcProgressNCTE(cur_path_seg.poses.front().pose, m_center_path));
        auto d_last = get<1>(calcProgressNCTE(cur_path_seg.poses.back().pose, m_center_path));
        bool is_path_cross_center = (d_first * d_last) < 0;
        /////////////////////////////////////
        // STATE CHANGE CONDITIONS
        /////////////////////////////////////
        if (naive_gap > 2 * m_small_gap && !overtake_start)
        {
          m_overtake_status = OVERTAKE_STATUS::BEFORE;
          m_overtake_decision_counter = 0;
          m_race_mode_first_callback = true;
        }
        else if (naive_gap > 20 && overtake_start && mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_STRAIGHT)
        {
          m_overtake_status = OVERTAKE_STATUS::DONE;
          m_overtake_decision_counter = 0;
        }
        else if (is_path_cross_center)
        {
          m_overtake_status = OVERTAKE_STATUS::ATTEMPT;
          m_overtake_decision_counter = 0;
        }
        else if (!cur_traj.has_collision)
        {
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
        }
        else
        {
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
        }
        break;
      }
      case OVERTAKE_STATUS::DONE:
        if (naive_gap < 15)
        {
          m_overtake_decision_counter++;
        }

        if (m_overtake_decision_counter > 50)
        {
          m_overtake_status = OVERTAKE_STATUS::ATTEMPT;
          m_overtake_decision_counter = 0;
        }
        else if ( // overtake success, merge to defender line @ corner
            mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_STRAIGHT &&
            naive_gap > m_merging_back_gap_thres && // longitudinal wise
            overtake_start &&
            m_last_update_target_path_alias != m_defenderline_file_path &&
            // m_last_update_target_path_alias != "left_raceline" &&
            // m_last_lat_planning_type == LATERAL_PLANNING_TYPE::KEEP &&
            m_reset_target_path_idx == RESET_PATH_TYPE::NONE)
        {
          // std::cout << "DONE" << std::endl;
          auto progressNCTE_defenderline =
              calcProgressNCTE(m_ego_odom.pose.pose, m_defenderline_path);
          auto frenet_time = std::max(abs(get<1>(progressNCTE_defenderline) /
                                          allowable_maximum_vy),
                                      default_planning_time_max);
          if (abs(get<1>(progressNCTE_defenderline)) < m_dist_same_lane_path_change)
          {
            frenet_time = std::max(abs(get<1>(progressNCTE_defenderline) /
                                       allowable_maximum_vy),
                                   default_planning_time_min);
          }
          // Merging frenet segment generation
          // Generate single frenet path segment
          std::vector<std::shared_ptr<FrenetPath>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                      get<1>(progressNCTE_defenderline), // current_position_d
                      get<0>(progressNCTE_defenderline), // current_position_s
                      0.0,                               // current_velocity_d
                      std::max(m_ego_odom.twist.twist.linear.x,
                               MIN_SPEED_MPS),            // current_velocity_s
                      0.0,                                // current_acceleration_d
                      get<4>(m_defenderline_spline_data), // cubicSplineModel
                      frenet_time,
                      frenet_time + 0.01,
                      SAMPLING_TIME, 0.0, 0.0001, 0.1);

          if (frenet_path_generation_result.empty() || // Abnormal situation
              frenet_path_generation_result[0]->points_x().empty())
          {
            // publish empty & Estop trajectory
            RCLCPP_ERROR_ONCE(
                this->get_logger(),
                "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                "generation result or point vector is empty");
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
            nif_msgs::msg::DynamicTrajectory empty_traj;
            empty_traj.header.frame_id =
                nif::common::frame_id::localization::ODOM;
            publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          auto stitched_path = stitchFrenetToPath(
              frenet_path_generation_result[0], m_defenderline_path);

          auto defenderline_path_seg = getCertainLenOfPathSeg(
              m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
              stitched_path.trajectory_path, 100);

          auto race_traj = m_velocity_profiler_obj.velProfileWCollisionChecking(
              m_ego_odom, defenderline_path_seg, m_cur_oppo_pred_result,
              m_config_overlap_checking_dist_bound,
              m_config_overlap_checking_time_bound, false, 1.0 /*, false*/); // @DEBUG

          if (!race_traj.has_collision)
          {
            // Change the defualt path to the racing line (full path)
            // Not considering the ACC in this case
            m_cur_planned_traj = stitched_path;
            m_reset_wpt_idx =
                getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                          frenet_path_generation_result[0]->points_y().back(),
                          m_cur_planned_traj.trajectory_path);

            m_last_update_target_path_alias = m_defenderline_file_path;
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::MERGE;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
            m_reset_target_path_idx = RESET_PATH_TYPE::DEFENDER_LINE;

            cur_traj = race_traj;
            m_overtake_decision_counter = 0;
          }
        }
        else if (!overtake_start && naive_gap > 10.0 && naive_gap < 70.0)
        {
          // std::cout << "debug10" << std::endl;
          m_overtake_decision_counter = 0;
          m_overtake_status = OVERTAKE_STATUS::BEFORE;
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
          m_race_mode_first_callback = true;
        }
        break;
      default:
        RCLCPP_ERROR_ONCE(
            this->get_logger(),
            "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n"
            "UNKNOWN overtake status");
        m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
        m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
        nif_msgs::msg::DynamicTrajectory empty_traj;
        empty_traj.header.frame_id =
            nif::common::frame_id::localization::ODOM;
        publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                 m_last_lat_planning_type, true);
        return;
      }
      publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                               m_last_lat_planning_type, true);

      // Publish status
      switch (m_overtake_status)
      {
      case OVERTAKE_STATUS::BEFORE:
        planner_status_msg.planning_code = planner_status_msg.ACC;
        break;
      case OVERTAKE_STATUS::ATTEMPT:
        planner_status_msg.planning_code = planner_status_msg.ATTEMPT;
        break;
      case OVERTAKE_STATUS::PASSING:
        planner_status_msg.planning_code = planner_status_msg.ATTEMPT;
        break;
      case OVERTAKE_STATUS::DONE:
        planner_status_msg.planning_code = planner_status_msg.OVERTAKE_DONE;
        break;
      }
      m_planner_status_pub->publish(planner_status_msg);
      // return;
    }

    else if (mission_status.mission_status_code ==
                 nif::common::MissionStatus::MISSION_CONSTANT_SPEED ||
             mission_status.mission_status_code ==
                 nif::common::MissionStatus::MISSION_SLOW_DRIVE)
    {
      // ----------------------------------------------
      // ---------------- Defender mode ---------------
      // --------------- Drive innerline --------------
      // ----------------- ACC activate ---------------
      // ----------------------------------------------

      m_race_mode_first_callback = true;
      m_overtake_status = OVERTAKE_STATUS::BEFORE;
      m_stop_mode_first_callback = true;
      m_non_overtaking_mode_first_callback = true;
      m_pit_mode_first_callback = true;
      if (m_last_update_target_path_alias == m_defenderline_file_path)
      {
        m_defender_mode_first_callback = false;
      }
      ///////////////////////////////
      // defender mode first callback
      ///////////////////////////////
      if ((m_defender_mode_first_callback == true && (mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_STRAIGHT || m_ego_odom.twist.twist.linear.x < 10.0)) || m_cur_planned_traj.trajectory_path.poses.empty())
      {
        m_defender_mode_first_callback = false;
        m_overtake_decision_counter = 0;

        // Switch to the defender line
        auto progressNCTE_defenderline =
            calcProgressNCTE(m_ego_odom.pose.pose, m_defenderline_path);
        auto frenet_time = std::max(abs(get<1>(progressNCTE_defenderline) /
                                        allowable_maximum_vy),
                                    default_planning_time_max);
        // std::cout << frenet_time << std::endl;
        if (abs(get<1>(progressNCTE_defenderline)) < m_dist_same_lane_path_change)
        {
          frenet_time = std::max(abs(get<1>(progressNCTE_defenderline) /
                                     allowable_maximum_vy),
                                 default_planning_time_min);
        }
        // std::cout<<abs(get<1>(progressNCTE_defenderline))<<std::endl;
        // std::cout<< frenet_time << std::endl;
        // Merging frenet segment generation
        // Generate SINGLE frenet path segment
        std::vector<std::shared_ptr<FrenetPath>>
            frenet_path_generation_result =
                m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                    get<1>(progressNCTE_defenderline), // current_position_d
                    get<0>(progressNCTE_defenderline), // current_position_s
                    0.0,                               // current_velocity_d
                    std::max(m_ego_odom.twist.twist.linear.x,
                             MIN_SPEED_MPS),            // current_velocity_s
                    0.0,                                // current_acceleration_d
                    get<4>(m_defenderline_spline_data), // cubicSplineModel
                    frenet_time,
                    frenet_time + 0.01,
                    SAMPLING_TIME, 0.0, 0.0001, 0.1);

        if (frenet_path_generation_result.empty() || // Abnormal situation
            frenet_path_generation_result[0]->points_x().empty())
        {
          // Abnormal situation
          // publish empty & Estop trajectory
          // @DEBUG:
          RCLCPP_ERROR_ONCE(this->get_logger(),
                            "CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                            "generation result or point vector is empty");
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
          nif_msgs::msg::DynamicTrajectory empty_traj;
          empty_traj.header.frame_id =
              nif::common::frame_id::localization::ODOM;
          publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                   m_last_lat_planning_type, true);
          return;
        }

        m_cur_planned_traj = stitchFrenetToPath(
            frenet_path_generation_result[0], m_defenderline_path);

        m_last_update_target_path_alias = m_defenderline_file_path;

        m_reset_wpt_idx =
            getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                      frenet_path_generation_result[0]->points_y().back(),
                      m_cur_planned_traj.trajectory_path);

        m_reset_target_path_idx = RESET_PATH_TYPE::DEFENDER_LINE;
      }
      // Keep previous plan

      ///////////////////////////////////////////
      // Change the target path to the static wpt
      ///////////////////////////////////////////
      auto cur_idx_on_previous_path = getCurIdx(
          m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
          m_cur_planned_traj.trajectory_path);

      checkSwitchToStaticWPT(cur_idx_on_previous_path);
      // -----------------------------------------

      auto cur_path_seg = getCertainLenOfPathSeg(
          m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
          m_cur_planned_traj.trajectory_path, 100);

      auto cur_traj = m_velocity_profiler_obj.velProfileWCollisionChecking(
          m_ego_odom, cur_path_seg, m_cur_oppo_pred_result,
          m_config_overlap_checking_dist_bound,
          m_config_overlap_checking_time_bound, false, 1.0 /*, true*/);

      bool oppo_is_front = true;
      auto naive_gap = nif::common::constants::numeric::INF;
      if (!m_cur_oppo_pred_result.trajectory_path.poses.empty())
      {
        naive_gap = nif::common::utils::geometry::calEuclideanDistance(
            m_ego_odom.pose.pose,
            m_cur_oppo_pred_result.trajectory_path.poses.front().pose);
        geometry_msgs::msg::PoseStamped oppo_pose_in_body = nif::common::utils::coordination::getPtGlobaltoBody(this->getEgoOdometry(),
                                                                                                                m_cur_oppo_pred_result.trajectory_path.poses.front().pose);
        oppo_is_front = oppo_pose_in_body.pose.position.x > 10;
      }
      auto d_first = get<1>(calcProgressNCTE(cur_path_seg.poses.front().pose, m_center_path));
      auto d_last = get<1>(calcProgressNCTE(cur_path_seg.poses.back().pose, m_center_path));
      double d_oppo = -d_first;
      bool is_path_cross_center = (d_first * d_last) < 0;
      if (!m_cur_oppo_pred_result.trajectory_path.poses.empty())
      {
        d_oppo = get<1>(calcProgressNCTE(m_cur_oppo_pred_result.trajectory_path.poses.front().pose, m_center_path));
      }
      bool oppo_in_same_lane = (d_first * d_oppo) > 0;
      // RIGHT OF WAY
      if (is_path_cross_center && !oppo_in_same_lane && (/*cur_traj.has_collision || */ naive_gap < 30.0) && mission_status.zone_status.zone_type != mission_status.zone_status.ZONE_TYPE_CORNER_MID)
      {
        // std::cout << "RIGHT OF WAY!!!!!!!!!" << std::endl;
        if (m_overtake_decision_counter <= 5)
        {
          m_overtake_decision_counter++;
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
        }
        else
        {
          cur_traj = findOvertakingPath(default_planning_time_min, allowable_maximum_vy, false);
          m_overtake_status = OVERTAKE_STATUS::PASSING;
          m_overtake_decision_counter = 0;
        }
      }
      // NO COLLISION
      else if (!cur_traj.has_collision && (naive_gap > 30.0) && mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_STRAIGHT)
      {
        m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
        m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
        if (m_overtake_decision_counter <= 20)
        {
          m_overtake_decision_counter++;
        }
        else
        {
          m_defender_mode_first_callback = true;
        }
      }
      else
      {
        m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
        m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
      }

      // Publish cur_traj
      // m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
      // m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
      publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                               m_last_lat_planning_type, true);

      planner_status_msg.planning_code = planner_status_msg.ACC;
      m_planner_status_pub->publish(planner_status_msg);
      // return;
    }
    // -------------------------------------------------------------------------------
    // STOP CASES
    //---------------------------------------------------------------------------------
    else if (this->missionIs(nif::common::MissionStatus::MISSION_STANDBY) ||
             this->missionIs(nif::common::MissionStatus::MISSION_COMMANDED_STOP) ||
             this->missionIs(nif::common::MissionStatus::MISSION_INIT) ||
             this->missionIs(nif::common::MissionStatus::MISSION_PIT_INIT) ||
             this->missionIs(nif::common::MissionStatus::MISSION_EMERGENCY_STOP))
    {
      // std::cout << "MISSION_EMERGENCY_STOP!!!!!!!!!!" << std::endl;
      
      m_defender_mode_first_callback = true;
      m_race_mode_first_callback = true;
      m_non_overtaking_mode_first_callback = true;
      m_pit_mode_first_callback == true;

      if (m_stop_mode_first_callback == true)
      {
        // std::cout << "debug1" << std::endl;
        m_stop_mode_first_callback = false;

        nav_msgs::msg::Path m_stopline_path;
        FrenetPathGenerator::CubicSpliner2DResult m_stopline_spline_data;
        // std::cout << "m_last_update_target_path_alias : " << m_last_update_target_path_alias << std::endl;
        if (m_last_update_target_path_alias == m_pitline_file_path)
        {
          m_stopline_path = m_pitline_path;
          m_stopline_spline_data = m_pitline_spline_data;
          // std::cout << "debug2" << std::endl;

        }
        // else if (m_last_update_target_path_alias == m_defenderline_file_path
        //   // || m_last_update_target_path_alias == "left_raceline"
        //   // || m_last_update_target_path_alias == "defenderline"
        //   || mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_CORNER_MID\
        //   || mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_CORNER_ENTRY\
        //   || mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_CORNER_EXIT){
        //   m_stopline_path = m_defenderline_path;
        //   m_stopline_spline_data = m_defenderline_spline_data;
        //   m_last_update_target_path_alias = m_defenderline_file_path;
        // }
        else if (m_last_update_target_path_alias == m_race_mode_file_path)
        {
          // std::cout << "debug3" << std::endl;
          m_stopline_path = m_racingline_path;
          m_stopline_spline_data = m_racingline_spline_data;
          m_last_update_target_path_alias = m_race_mode_file_path;
        }
        else if (m_last_update_target_path_alias == m_defenderline_file_path)
        {
          // std::cout << "defender" << std::endl;

          m_stopline_path = m_defenderline_path;
          m_stopline_spline_data = m_defenderline_spline_data;
          m_last_update_target_path_alias = m_defenderline_file_path;
        }
        else if (m_last_update_target_path_alias == m_staybehind_file_path)
        {
          // std::cout << "staybehind" << std::endl;
          m_stopline_path = m_staybehind_path;
          m_stopline_spline_data = m_staybehind_spline_data;
          m_last_update_target_path_alias = m_staybehind_file_path;
        }
        // std::cout << "debug55" << std::endl;

        auto progreeNCTE_stopline =
            calcProgressNCTE(m_ego_odom.pose.pose, m_stopline_path);
        // std::cout << "calc" << std::endl;
           
        // Merging frenet segment generation
        // Generate SINGLE frenet path segment
        std::vector<std::shared_ptr<FrenetPath>>
            frenet_path_generation_result =
                m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                    get<1>(progreeNCTE_stopline), // current_position_d
                    get<0>(progreeNCTE_stopline), // current_position_progreeNCTE_stoplines
                    0.0,                          // current_velocity_d
                    std::max(m_ego_odom.twist.twist.linear.x,
                             MIN_SPEED_MPS),                        // current_velocity_s
                    0.0,                                            // current_acceleration_d
                    get<4>(m_stopline_spline_data),                 // cubicSplineModel
                    2 * std::max(abs(get<1>(progreeNCTE_stopline) / // extra time for smooth frenet path
                                     allowable_maximum_vy),
                                 default_planning_time_max),
                    2 * std::max(abs(get<1>(progreeNCTE_stopline) / // extra time for smooth frenet path
                                     allowable_maximum_vy),
                                 default_planning_time_max) +
                        0.01,
                    SAMPLING_TIME, 0.0, 0.0001, 0.1);
        // std::cout << "debug6" << std::endl;

        if (frenet_path_generation_result.empty() ||
            frenet_path_generation_result[0]->points_x().empty())
        {
          // Abnormal situation
          // publish empty & Estop trajectory
          // @DEBUG:
          RCLCPP_ERROR_ONCE(this->get_logger(),
                            "CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                            "generation result or point vector is empty");
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
          nif_msgs::msg::DynamicTrajectory empty_traj;
          empty_traj.header.frame_id =
              nif::common::frame_id::localization::ODOM;
          publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                   m_last_lat_planning_type, true);
          return;
        }
        // std::cout << "debug4" << std::endl;


        m_cur_planned_traj = stitchFrenetToPath(frenet_path_generation_result[0], m_stopline_path);
      }
      // std::cout << "debug5" << std::endl;

      auto cur_path_seg = getCertainLenOfPathSeg(
          m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
          m_cur_planned_traj.trajectory_path, 100);
      // std::cout << "debug6" << std::endl;

      // Convert maptrack to trajectory and publish (only global / without
      // ACC)
      auto cur_traj = m_velocity_profiler_obj.velProfile(
          m_ego_odom, cur_path_seg, 1.0);
      // std::cout << "debug7" << std::endl;

      m_last_lat_planning_type = LATERAL_PLANNING_TYPE::CHANGE_PATH;
      m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
      publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                               m_last_lat_planning_type, true);

      planner_status_msg.planning_code = planner_status_msg.STOP;
      m_planner_status_pub->publish(planner_status_msg);
      // return;
    }
    else
    {
      // std::cout << "KEEP_FOLLOW" << std::endl;

      // KEEP FOLLOW
      ////////////////////////////
      // Overtaking is not allowed
      // Stay behind
      ////////////////////////////

      m_defender_mode_first_callback = true;
      m_race_mode_first_callback = true;
      m_overtake_status = OVERTAKE_STATUS::BEFORE;
      m_stop_mode_first_callback = true;
      m_pit_mode_first_callback = true;

      if ((m_non_overtaking_mode_first_callback == true
           // && mission_status.zone_status.zone_type != mission_status.zone_status.ZONE_TYPE_CORNER_MID
           && (mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_STRAIGHT || m_ego_odom.twist.twist.linear.x < 10.0)) ||
          m_cur_planned_traj.trajectory_path.poses.empty())
      {
        m_non_overtaking_mode_first_callback = false;

        auto progressNCTE_staybehind =
            calcProgressNCTE(m_ego_odom.pose.pose, m_staybehind_path);
        auto frenet_time = std::max(abs(get<1>(progressNCTE_staybehind) /
                                        allowable_maximum_vy),
                                    default_planning_time_max);
        if (abs(get<1>(progressNCTE_staybehind)) < m_dist_same_lane_path_change)
        {
          frenet_time = std::max(abs(get<1>(progressNCTE_staybehind) /
                                     allowable_maximum_vy),
                                 default_planning_time_min);
        }
        // Merging frenet segment generation
        // Generate single frenet path segment
        std::vector<std::shared_ptr<FrenetPath>>
            frenet_path_generation_result =
                m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                    get<1>(progressNCTE_staybehind), // current_position_d
                    get<0>(progressNCTE_staybehind), // current_position_s
                    0.0,                             // current_velocity_d
                    std::max(m_ego_odom.twist.twist.linear.x,
                             MIN_SPEED_MPS),          // current_velocity_s
                    0.0,                              // current_acceleration_d
                    get<4>(m_staybehind_spline_data), // cubicSplineModel
                    frenet_time,
                    frenet_time + 0.01,
                    SAMPLING_TIME, 0.0, 0.0001, 0.1);

        if (frenet_path_generation_result.empty() ||
            frenet_path_generation_result[0]->points_x().empty())
        {

          // Abnormal situation
          // publish empty & Estop trajectory
          RCLCPP_ERROR_ONCE(this->get_logger(),
                            "[MISSION_KEEP_POSITION] CRITICAL BUG HAS BEEN "
                            "HIT.\n Frenet path "
                            "generation result or point vector is empty");
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
          nif_msgs::msg::DynamicTrajectory empty_traj;
          empty_traj.header.frame_id =
              nif::common::frame_id::localization::ODOM;
          publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                   m_last_lat_planning_type, true);
          return;
        }

        auto stitched_path = stitchFrenetToPath(
            frenet_path_generation_result[0], m_staybehind_path);

        m_cur_planned_traj = stitched_path;

        m_reset_wpt_idx =
            getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                      frenet_path_generation_result[0]->points_y().back(),
                      m_cur_planned_traj.trajectory_path);

        // TODO: this should be the same with staybehind!!!!!!!!!!!!!!!!
        m_last_update_target_path_alias = m_staybehind_file_path;
        m_reset_target_path_idx = RESET_PATH_TYPE::STAY_BEHIND;

        // m_last_update_target_path_alias = "raceline";
        // m_reset_target_path_idx = RESET_PATH_TYPE::RACE_LINE;
      }

      // Keep previous plan
      ///////////////////////////////////////////
      // Change the target path to the static wpt
      ///////////////////////////////////////////
      auto cur_idx_on_previous_path = getCurIdx(
          m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
          m_cur_planned_traj.trajectory_path);

      checkSwitchToStaticWPT(cur_idx_on_previous_path);
      // -----------------------------------------

      //////////////////////////////////////////////////////////////////
      // Merging to the right side if the ego vehicle is close from the
      // opponent (only in the straight section --from mission mananger)
      //////////////////////////////////////////////////////////////////

      // auto naive_gap = nif::common::constants::numeric::INF;
      // if (!m_cur_oppo_pred_result.trajectory_path.poses.empty()) {
      //   naive_gap = nif::common::utils::geometry::calEuclideanDistance(
      //       m_ego_odom.pose.pose,
      //       m_cur_oppo_pred_result.trajectory_path.poses.front().pose);
      // }

      // // Check merging behavior only on the straight section
      // if (mission_status.zone_status.zone_type == mission_status.zone_status.ZONE_TYPE_STRAIGHT &&
      //     naive_gap < 150 &&
      //     m_last_update_target_path_alias != "right_center" &&
      //     m_last_update_target_path_alias != "stay_behind" &&
      //     m_reset_target_path_idx == RESET_PATH_TYPE::NONE) {

      //   auto progreeNCTE_staybehind =
      //       calcProgressNCTE(m_ego_odom.pose.pose, m_staybehind_path);

      //   // Merging frenet segment generation
      //   // Generate single frenet path segment
      //   std::vector<std::shared_ptr<FrenetPath>>
      //       frenet_path_generation_result =
      //           m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
      //               get<1>(progreeNCTE_staybehind), // current_position_d
      //               get<0>(progreeNCTE_staybehind), // current_position_s
      //               0.0,                            // current_velocity_d
      //               std::max(m_ego_odom.twist.twist.linear.x,
      //                         MIN_SPEED_MPS), // current_velocity_s
      //               0.0,                     // current_acceleration_d
      //               get<4>(m_staybehind_spline_data), // cubicSplineModel
      //               std::max(abs(get<1>(progreeNCTE_staybehind) /
      //                             allowable_maximum_vy),
      //                         default_planning_time_max),
      //               std::max(abs(get<1>(progreeNCTE_staybehind) /
      //                             allowable_maximum_vy),
      //                         default_planning_time_max) +
      //                   0.01,
      //               SAMPLING_TIME, 0.0, 0.0001, 0.1);

      //   if (frenet_path_generation_result.empty() ||
      //       frenet_path_generation_result[0]->points_x().empty()) {
      //     // Abnormal situation
      //     // publish empty & Estop trajectory
      //     RCLCPP_ERROR_ONCE(
      //         this->get_logger(),
      //         "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n Frenet path "
      //         "generation result or point vector is empty");
      //     m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
      //     m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
      //     nif_msgs::msg::DynamicTrajectory empty_traj;
      //     empty_traj.header.frame_id =
      //         nif::common::frame_id::localization::ODOM;
      //     publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
      //                               m_last_lat_planning_type, true);
      //     return;
      //   }

      //   auto stitched_path = stitchFrenetToPath(
      //       frenet_path_generation_result[0], m_staybehind_path);

      //   auto staybehind_path_seg = getCertainLenOfPathSeg(
      //       m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
      //       stitched_path.trajectory_path, 100);

      //   auto staybehind_traj =
      //       m_velocity_profiler_obj.velProfileWCollisionChecking(
      //           m_ego_odom, staybehind_path_seg, m_cur_oppo_pred_result,
      //           m_config_overlap_checking_dist_bound,
      //           m_config_overlap_checking_time_bound, false, 1.0); // @DEBUG

      //   if (!staybehind_traj.has_collision) {
      //     // Change the defualt path to the racing line (full path)
      //     // Not considering the ACC in this case
      //     m_cur_planned_traj = stitched_path;
      //     m_reset_wpt_idx =
      //         getCurIdx(frenet_path_generation_result[0]->points_x().back(),
      //                   frenet_path_generation_result[0]->points_y().back(),
      //                   m_cur_planned_traj.trajectory_path);

      //     m_last_update_target_path_alias = "stay_behind";
      //     m_last_lat_planning_type = LATERAL_PLANNING_TYPE::MERGE;
      //     m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
      //     m_reset_target_path_idx = RESET_PATH_TYPE::STAY_BEHIND;

      //     publishPlannedTrajectory(staybehind_traj, m_last_long_planning_type,
      //                               m_last_lat_planning_type, true);
      //     return;
      //   }
      // }

      // Keep previous plan and ACC trajectory generation
      auto cur_path_seg = getCertainLenOfPathSeg(
          m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
          m_cur_planned_traj.trajectory_path, 100);

      auto cur_traj = m_velocity_profiler_obj.velProfileForAcc(
          m_ego_odom, m_cur_oppo_pred_result,
          m_cur_det_global.obj_velocity_in_global.linear.x, cur_path_seg,
          1.0);

      // Publish cur_traj
      m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
      m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
      publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                               m_last_lat_planning_type, true);

      planner_status_msg.planning_code = planner_status_msg.ACC;
      m_planner_status_pub->publish(planner_status_msg);
      // return;
    }
    // planner_status_msg.callback_delay = (double)((this->now() - temp_time).nanoseconds());
    m_planner_status_pub->publish(planner_status_msg);
    // return;
  }
  else
  {
    // ESTOP
    nif::common::NodeStatusCode node_status = nif::common::NODE_INITIALIZED;
    this->setNodeStatus(node_status);
  }
  auto time_diff = (double)((this->now() - temp_time).nanoseconds());
  planner_status_msg.callback_delay = pow(10, 9) / time_diff;
  // std::cout << pow(10, 9) / planner_status_msg.callback_delay << std::endl;
  m_planner_status_pub->publish(planner_status_msg);
}