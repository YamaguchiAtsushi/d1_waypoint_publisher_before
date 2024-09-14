#ifndef D1_WAYPOINT_PUBLISHER_CORE_HPP_
#define D1_WAYPOINT_PUBLISHER_CORE_HPP_


#include <cmath>
#include <stdexcept>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"


#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include "tf2/LinearMath/Quaternion.h"

#define FOLLOW_WAYPOINTS_MODE 1
#define THROUGH_POSES_MODE 0

#define SEND_WAYPOINTS 0
#define SEND_WAYPOINTS_CHECK 1
#define WAITING_GOAL 2
#define WAITING_BUTTON 3
#define FINISH_SENDING 4
#define ROTATION 5
#define SEND_WAYPOINTS1 6
#define SEND_WAYPOINTS2 7
#define SEND_WAYPOINTS3 8
#define SEND_WAYPOINTS4 9
#define APPROACH_POINT 10
#define APPROACH_AREA 11




using namespace std::chrono_literals;

typedef struct{
  geometry_msgs::msg::Pose poses;
  bool will_stop;
}waypoint_info;


using std::placeholders::_1;
using std::placeholders::_2;

class D1WaypointPublisher : public rclcpp::Node{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
  //rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
  D1WaypointPublisher();

private:
  std::vector<std::string> getCSVLine(std::string& input, char delimiter);
  void ReadWaypointsFromCSV(std::string& csv_file, std::vector<waypoint_info>& waypoints);
  void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result);
  void NavThroughPosesResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result);
  void NavThroughPosesGoalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>> future);
  void NavThroughPosesFeedbackCallback(const GoalHandleNavigateNavigateThroughPoses::SharedPtr,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
  void SendWaypointsTimerCallback();
  void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void AreaCallback(const std_msgs::msg::String::SharedPtr msg);
  size_t SendWaypointsOnce(size_t sending_index);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr area_sub_;


  geometry_msgs::msg::Pose current_pose_;
  std_msgs::msg::String area_character_;
  //geometry_msgs::msg::Twist twist_msg;
  


  rclcpp::TimerBase::SharedPtr timer_;
  int id_;
  //std::string csv_file_;
  std::string waypoints_file_1_;
  std::string waypoints_file_2_;
  std::string waypoints_file_3_;
  std::string waypoints_file_4_;
  std::string waypoints_file_5_;
  std::string waypoints_file_6_;
  
  std::vector<std::string> csv_file_;//自分で追加
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_goal_handle_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions send_goal_options_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_goal_handle_;
  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>>> future_goal_handle_;

  std::vector<waypoint_info> waypoints_;
  size_t start_index_;

  int find_point_;//初期化してない
  int find_character_;
  int goal_point_;//初期化してない


  int follow_type_;
  int start_index_int_;
  bool is_action_server_ready_;
  bool is_goal_achieved_;
  bool is_aborted_;
  bool is_standby_;
  bool is_goal_accepted_;

  int send_waypoint2_flag;
  int send_waypoint3_flag;
  int send_waypoint4_flag;
  int approach_area_flag;

  double current_yaw_;

  double goal_x;
  double goal_y;

  int16_t number_of_poses_remaining_;
};
#endif