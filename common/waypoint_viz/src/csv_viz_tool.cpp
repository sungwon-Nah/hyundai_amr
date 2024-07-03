#include <chrono>
#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <string>
#include <vector>
#include <tuple>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <nif_common/types.h>
#include <nav_msgs/msg/path.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rcutils/error_handling.h"
#include <nif_common/constants.h>
#include <nif_common/types.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;
using namespace std;

class CsvVizTool : public rclcpp::Node
{
  public:
    CsvVizTool()
    : Node("csv_visualization_node")
    {
      this->declare_parameter("track_name", "LVMS");

      track = this->get_parameter("track_name").as_string();

      m_inner_boundary_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/inner_boundary", 10);
      m_outer_boundary_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/outer_boundary", 10);
      m_pit_inner_boundary_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/pit_inner_boundary", 10);
      m_pit_outer_boundary_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/pit_outer_boundary", 10);

      m_raceline_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/raceline", 10);
      m_left_raceline_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/left_raceline", 10);
      m_right_raceline_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/right_raceline", 10);
      m_left_center_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/left_center", 10);
      m_right_center_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/right_center", 10);
      m_pit_lane_pub  = this->create_publisher<nav_msgs::msg::Path>("/vis/pit_lane", 10);
      m_extra_pub = this->create_publisher<nav_msgs::msg::Path>("/vis/centerline", 10);
  // timer
      timer_ = this->create_wall_timer(
      100ms, std::bind(&CsvVizTool::timer_callback, this));

      try {
      m_path_to_maps_dir = ament_index_cpp::get_package_share_directory("nif_waypoint_manager_nodes");
      std::cout << m_path_to_maps_dir << std::endl;
      } catch (std::exception e) {
        RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
      }

      m_path_to_maps_dir.append("/maps/");
      m_path_to_maps_dir.append(track);
      m_path_to_maps_dir.append("/");
      m_inner_boundary_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "trackboundary_left.csv"));
      m_outer_boundary_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "trackboundary_right.csv"));
      // try { m_pit_inner_boundary_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "pit_trackboundary_left.csv"));}
      // catch (std::exception e){}
      // try { m_pit_outer_boundary_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "pit_trackboundary_right.csv"));}
      // catch (std::exception e){}
      m_raceline_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "race_line.csv"));
      try {m_left_raceline_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "left_center.csv"));}
      catch (std::exception e){}
      try {m_right_raceline_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "right_center.csv"));}
      catch (std::exception e){}
      m_left_center_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "left_center.csv"));
      m_right_center_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "right_center.csv"));
      m_pit_lane_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "pit_lane.csv"));
      m_extra_msg = this->xyVec2Path(this->loadCSVfile(m_path_to_maps_dir + "centerline.csv"));
  }

  private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_inner_boundary_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_outer_boundary_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pit_inner_boundary_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pit_outer_boundary_pub;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_raceline_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_left_raceline_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_right_raceline_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_left_center_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_right_center_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pit_lane_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_extra_pub;
    std::string m_path_to_maps_dir;

    nav_msgs::msg::Path m_inner_boundary_msg = nav_msgs::msg::Path();
    nav_msgs::msg::Path m_outer_boundary_msg = nav_msgs::msg::Path();
    nav_msgs::msg::Path m_pit_inner_boundary_msg = nav_msgs::msg::Path();
    nav_msgs::msg::Path m_pit_outer_boundary_msg = nav_msgs::msg::Path();

    nav_msgs::msg::Path m_raceline_msg = nav_msgs::msg::Path();
    nav_msgs::msg::Path m_left_raceline_msg = nav_msgs::msg::Path();
    nav_msgs::msg::Path m_right_raceline_msg = nav_msgs::msg::Path();
    nav_msgs::msg::Path m_left_center_msg = nav_msgs::msg::Path();
    nav_msgs::msg::Path m_right_center_msg = nav_msgs::msg::Path();
    nav_msgs::msg::Path m_pit_lane_msg = nav_msgs::msg::Path();
    nav_msgs::msg::Path m_extra_msg = nav_msgs::msg::Path();

    rclcpp::TimerBase::SharedPtr timer_;
    
    string track = "";

    tuple<vector<double>, vector<double>> loadCSVfile(const std::string &wpt_file_path_);
    nav_msgs::msg::Path xyVec2Path(tuple<std::vector<double>, std::vector<double>> xy);

    void timer_callback()
    {
      // m_inner_boundary_msg.header.stamp = rclcpp::Clock().now();                
      // m_outer_boundary_msg.header.stamp = rclcpp::Clock().now(); 
      m_inner_boundary_pub->publish(m_inner_boundary_msg);
      m_outer_boundary_pub->publish(m_outer_boundary_msg);
      m_pit_inner_boundary_pub->publish(m_pit_inner_boundary_msg);
      m_pit_outer_boundary_pub->publish(m_pit_outer_boundary_msg);

      m_raceline_pub->publish(m_raceline_msg);
      m_left_raceline_pub->publish(m_left_raceline_msg);
      m_right_raceline_pub->publish(m_right_raceline_msg);
      m_left_center_pub->publish(m_left_center_msg);
      m_right_center_pub->publish(m_right_center_msg);
      m_pit_lane_pub->publish(m_pit_lane_msg);
      m_extra_pub->publish(m_extra_msg);
    }


  

};

tuple<vector<double>, vector<double>> 
CsvVizTool::loadCSVfile(const std::string &wpt_file_path_) {
  ifstream inputFile(wpt_file_path_);
  vector<double> vec_x, vec_y;

  while (inputFile) {
    string s;
    if (!getline(inputFile, s)) {
      break;
    }
    if (s[0] != '#') {
      istringstream ss(s);
      int cnt = 0;
      bool nan_flg = false;
      while (ss) {
        string line;
        if (!getline(ss, line, ',')) {
          break;
        }
        try {
          if (cnt == 0) {
            vec_x.push_back(stof(line));
          } else if (cnt == 1) {
            vec_y.push_back(stof(line));
          }
        }
        catch (const invalid_argument e) {
          cout << "NaN found in file " << wpt_file_path_ << endl;
          e.what();
          nan_flg = true;
        }
        cnt++;
      }
    }
  }

  if (!inputFile.eof()) {
    cerr << "Could not read file " << wpt_file_path_ << "\n";
    __throw_invalid_argument("File not found.");
  }

  if (vec_x.size() == 0 || vec_y.size() == 0 ||
      (vec_x.size() != vec_y.size())) {
    __throw_invalid_argument("WPT SIZE ERROR.");
  }

  return std::make_tuple(vec_x, vec_y);
}

nav_msgs::msg::Path
CsvVizTool::xyVec2Path(tuple<std::vector<double>, std::vector<double>> xy) {
  nav_msgs::msg::Path output;
  output.header.frame_id = nif::common::constants::parameters::VALUE_GLOBAL_FRAME_ID;
  auto x_ = get<0>(xy);
  auto y_ = get<1>(xy);
  for (int i = 0; i < x_.size(); i++) {
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.position.x = x_[i];
    pt.pose.position.y = y_[i];
    pt.pose.position.z = 0.0;

    // orientation
    int next_idx = (i+1) % x_.size();
    double next_x = x_[next_idx];
    double next_y = y_[next_idx];
    tf2::Quaternion quat_ekf;
    double yaw = atan2(next_y - pt.pose.position.y, next_x - pt.pose.position.x);
    if (std::isnan(yaw)){
        yaw = 0;
    }
    quat_ekf.setRPY(0,0,yaw);
    quat_ekf.normalize();
    pt.pose.orientation = tf2::toMsg(quat_ekf);

    output.poses.push_back(pt);
  }

  return output;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvVizTool>());
  rclcpp::shutdown();
  return 0;
}