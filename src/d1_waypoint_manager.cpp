#include "d1_waypoint_manager/d1_waypoint_manager.hpp"  // include local header

D1WaypointPublisher::D1WaypointPublisher() : rclcpp::Node("d1_waypoint_manager"), id_(0)
{
  //std::cout << "1" << std::endl;

  //follow_type_ = FOLLOW_WAYPOINTS_MODE;//自分でつけた
  follow_type_ = THROUGH_POSES_MODE;//自分でつけた

  rclcpp::QoS latched_qos{ 1 };
  latched_qos.transient_local();
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("pose", 10, std::bind(&D1WaypointPublisher::PoseCallback, this, std::placeholders::_1));
  area_sub_ = create_subscription<std_msgs::msg::String>("area", 10, std::bind(&D1WaypointPublisher::AreaCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&D1WaypointPublisher::OdomCallback, this, std::placeholders::_1));


  std::string action_server_name;

  if (follow_type_ == THROUGH_POSES_MODE){
    action_server_name = "navigate_through_poses";
    nav_through_poses_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, action_server_name);
    rclcpp::sleep_for(500ms);
    is_action_server_ready_ = nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));

  }else if (follow_type_ == FOLLOW_WAYPOINTS_MODE){
    action_server_name = "follow_waypoints";
    follow_waypoints_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, action_server_name);
    rclcpp::sleep_for(500ms);
    is_action_server_ready_ = follow_waypoints_action_client_->wait_for_action_server(std::chrono::seconds(5));
  }

  send_goal_options_ = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options_.feedback_callback = std::bind(&D1WaypointPublisher::NavThroughPosesFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options_.result_callback = std::bind(&D1WaypointPublisher::NavThroughPosesResultCallback, this, std::placeholders::_1);
  send_goal_options_.goal_response_callback = std::bind(&D1WaypointPublisher::NavThroughPosesGoalResponseCallback, this, std::placeholders::_1);

  waypoints_file_1_ = "/home/yamaguchi-a/turtlebot3_ws/src/d1_waypoint_manager/csv/waypoints_1.csv";//自分でつけた
  waypoints_file_2_ = "/home/yamaguchi-a/turtlebot3_ws/src/d1_waypoint_manager/csv/waypoints_2.csv";//自分でつけた
  waypoints_file_3_ = "/home/yamaguchi-a/turtlebot3_ws/src/d1_waypoint_manager/csv/waypoints_3.csv";//自分でつけた
  waypoints_file_4_ = "/home/yamaguchi-a/turtlebot3_ws/src/d1_waypoint_manager/csv/waypoints_a.csv";//自分でつけた
  waypoints_file_5_ = "/home/yamaguchi-a/turtlebot3_ws/src/d1_waypoint_manager/csv/waypoints_b.csv";//自分でつけた
  waypoints_file_6_ = "/home/yamaguchi-a/turtlebot3_ws/src/d1_waypoint_manager/csv/waypoints_c.csv";//自分でつけた


  csv_file_ = {waypoints_file_1_, waypoints_file_2_, waypoints_file_3_, waypoints_file_4_, waypoints_file_5_, waypoints_file_6_,};//自分で追加

  send_waypoint2_flag = 0;
  send_waypoint3_flag = 0;
  send_waypoint4_flag = 0;
  approach_area_flag = 0;

  // ロボットの現在位置と向きの初期化
  current_pose_.position.x = 0.0;
  current_pose_.position.y = 0.0;
  current_yaw_ = 0.0;

  ReadWaypointsFromCSV(csv_file_[0], waypoints_);
  start_index_ = (size_t)start_index_int_;
  start_index_ = 1; //自分でつけた
  if(start_index_ < 1 || start_index_ > waypoints_.size()){
    RCLCPP_ERROR(get_logger(), "Invalid start_index");
    std::cout << "start_index_:" << start_index_ << "waypoints_.size()" << waypoints_.size() << std::endl;
    return;
  }

  if (is_action_server_ready_){
     timer_ = create_wall_timer(100ms, std::bind(&D1WaypointPublisher::SendWaypointsTimerCallback, this));
  }else{
    RCLCPP_ERROR(this->get_logger(),
                  "%s action server is not available."
                  " Is the initial pose set?"
                  " SendWaypoints was not executed.",
                  action_server_name.c_str());
    return;
  }
}

geometry_msgs::msg::Quaternion rpyYawToQuat(double yaw){
  tf2::Quaternion tf_quat;
  geometry_msgs::msg::Quaternion msg_quat;
  tf_quat.setRPY(0.0, 0.0 ,yaw);
  msg_quat = tf2::toMsg(tf_quat);
  return msg_quat;
}

std::vector<std::string> D1WaypointPublisher::getCSVLine(std::string& input, char delimiter)
{
  //std::cout << "2" << std::endl;
  
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (getline(stream, field, delimiter))
  {
    result.push_back(field);
  }
  return result;
}

void D1WaypointPublisher::ReadWaypointsFromCSV(std::string& csv_file, std::vector<waypoint_info>& waypoints)
{
  //std::cout << "3" << std::endl;

  std::ifstream ifs(csv_file);
  std::string line;
  waypoint_info waypoint;

  while (getline(ifs, line))
  {
    id_++;
    std::vector<std::string> strvec = getCSVLine(line, ',');
    waypoint.poses.position.x = std::stod(strvec.at(0));
    waypoint.poses.position.y = std::stod(strvec.at(1));
    waypoint.poses.position.z = 0.0;
    waypoint.poses.orientation = rpyYawToQuat(std::stod(strvec.at(2))/180.0*M_PI);
    waypoint.will_stop = ("1"==strvec.at(5));

    // std::cout << "-------------------------------------" << std::endl;
    // std::cout << "waypoint ID: " << id << std::endl;
    // std::cout << "trans x: " << std::stod(strvec.at(0)) << std::endl;
    // std::cout << "trans y: " << std::stod(strvec.at(1)) << std::endl;
    // std::cout << "rot yaw: " << std::stod(strvec.at(2)) << std::endl;
    // std::cout << "will stop: "<< std::boolalpha <<  ("1"==strvec.at(5)) << std::endl;

    waypoints.push_back(waypoint);
  }
}


void D1WaypointPublisher::SendWaypointsTimerCallback(){
  std::cout << "4" << std::endl;
  static size_t sending_index = start_index_ - 1;
  //static int state = SEND_WAYPOINTS;
  static int state = SEND_WAYPOINTS1;
  //static int state = APPROACH_POINT;

  std::cout << "sending_index" << sending_index << std::endl;

  switch (state)
  {
  case SEND_WAYPOINTS1:
    std::cout << "SEND_WAYPOINYS1" << std::endl;
    //std::cout << "find_point_" << find_point_ << std::endl; 
    std::cout << "find_character_" << find_character_ << std::endl; 

    if(sending_index < waypoints_.size()){
      sending_index =  SendWaypointsOnce(sending_index);
    }
    if(is_goal_achieved_){//trueになったら
        //if(find_character_ == 1){
          //is_goal_achieved_ = false;
          //is_goal_accepted_ = false;
          //is_aborted_ = false;
          //state = APPROACH_AREA;
          //std::cout << "APPROACH_POINT start" << std::endl;
        //}
        if(find_point_ == 1){
          is_goal_achieved_ = false;
          is_goal_accepted_ = false;
          is_aborted_ = false;
          state = APPROACH_POINT;
          //state = APPROACH_AREA;
          std::cout << "APPROACH_POINT start" << std::endl;
        }
        else{
          state = SEND_WAYPOINTS2;
        }
  }
    break;

  case SEND_WAYPOINTS2:
    std::cout << "SEND_WAYPOINYS2" << std::endl;
    if (send_waypoint2_flag == 0){
      ReadWaypointsFromCSV(csv_file_[1], waypoints_);
    }
    send_waypoint2_flag = 1;
    //sending_index = 0; //いらないかも
    if(sending_index < waypoints_.size()){
      sending_index =  SendWaypointsOnce(sending_index);
    }
    if (is_goal_achieved_) {  // waypoint2に到達したら
      state = SEND_WAYPOINTS3;
      is_goal_achieved_ = false;
      is_goal_accepted_ = false;
      is_aborted_ = false;
    }
    break;

  case SEND_WAYPOINTS3:
    std::cout << "SEND_WAYPOINYS3" << std::endl;
    if (send_waypoint3_flag == 0){
      ReadWaypointsFromCSV(csv_file_[2], waypoints_);
    }
    send_waypoint3_flag = 1;
    //sending_index = 0; //いらないかも
    if(sending_index < waypoints_.size()){
      sending_index =  SendWaypointsOnce(sending_index);

    }
    if (is_goal_achieved_) {  // waypoint2に到達したら
      state = SEND_WAYPOINTS4;
      is_goal_achieved_ = false;
      is_goal_accepted_ = false;
      is_aborted_ = false;
    }
    break;

  case FINISH_SENDING:
    RCLCPP_INFO(this->get_logger(), "Waypoint sending is Finisihed.");
    timer_->cancel();
    state = APPROACH_POINT;
    break;

  case APPROACH_POINT:
    //twist_pub_->publish(twist_msg);
    std::cout << "APPROACH_POINT running" << std::endl;
    if(goal_point_ == 1){
      if(find_character_ == 1){
        state = APPROACH_AREA;
      }
    }
    break;

  case APPROACH_AREA:
    std::cout << "APPROACH_AREA running" << std::endl;
    //以下、SEND_WAYPOINTの改変
    //一度しかwaypointを読み込まないようにしている
    if (approach_area_flag == 0 ){
      if(area_character_.data == "a"){
        ReadWaypointsFromCSV(csv_file_[3], waypoints_);
      }
      if(area_character_.data == "b"){
        std::cout << "b received" << std::endl;
        ReadWaypointsFromCSV(csv_file_[4], waypoints_);
      }
      if(area_character_.data == "c"){
        std::cout << "c received" << std::endl;
        ReadWaypointsFromCSV(csv_file_[5], waypoints_);
      }
      
    }
    approach_area_flag = 1;
    //sending_index = 0; //いらないかも
    if(sending_index < waypoints_.size()){
      sending_index =  SendWaypointsOnce(sending_index);
        std::cout << "sendwaypointsonce after c received" << std::endl;

    }
    if (is_goal_achieved_) {  // waypoint2に到達したら
      state = SEND_WAYPOINTS3;
      is_goal_achieved_ = false;
      is_goal_accepted_ = false;
      is_aborted_ = false;
    }
    break;

  //case EXIT:
  //  std::cout << "EXIT running" << std::endl;



    
  default:
    RCLCPP_INFO(this->get_logger(), "UNKNOWN ERROR");
    timer_->cancel();
    break;
  }
}

size_t D1WaypointPublisher::SendWaypointsOnce(size_t sending_index){
  //std::cout << "5" << std::endl;
    std::cout << "sending_index" << sending_index << std::endl;


  size_t i;
  size_t next_index;
  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal;
  nav2_msgs::action::FollowWaypoints::Goal follow_waypoints_goal;
  for(i = sending_index; i<waypoints_.size(); i++){
    //std::cout << "5-1" << std::endl;

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose=waypoints_[i].poses;
    nav_through_poses_goal.poses.push_back(goal_msg);
    follow_waypoints_goal.poses.push_back(goal_msg);
    if(waypoints_[i].will_stop)break;
  }
  next_index = i + 1;
  if (follow_type_ == THROUGH_POSES_MODE){
    is_goal_achieved_ = false;
    is_goal_accepted_ = false;
    is_aborted_ = false;

    std::chrono::milliseconds server_timeout(1000);
    future_goal_handle_ = nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal, send_goal_options_);
    RCLCPP_INFO(this->get_logger(),
              "[nav_through_poses]: Sending a path of %zu waypoints:", nav_through_poses_goal.poses.size());
  }

  if (follow_type_ == FOLLOW_WAYPOINTS_MODE){
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) { follow_waypoints_goal_handle_.reset(); };

    std::chrono::milliseconds server_timeout(1000);
    auto future_goal_handle =
        follow_waypoints_action_client_->async_send_goal(follow_waypoints_goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, server_timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
      //return;
    }
    // Get the goal handle and save so that we can check on completion in the timer callback
    follow_waypoints_goal_handle_ = future_goal_handle.get();
    if (!follow_waypoints_goal_handle_)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    RCLCPP_INFO(this->get_logger(),
                "[follow_waypoints]: Sending a path of %zu waypoints:", follow_waypoints_goal.poses.size());
  }
  return next_index;
}




void D1WaypointPublisher::NavThroughPosesResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result){
  //std::cout << "6" << std::endl;

  nav_through_poses_goal_handle_.reset();
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      is_goal_achieved_ = true;
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      is_aborted_ = true;
      return;
    
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
}
void D1WaypointPublisher::NavThroughPosesGoalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>> future){
  //std::cout << "7" << std::endl;

  auto handle = future.get();
  if (!handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    timer_->cancel();
    return ;
  }
  is_goal_accepted_ = true; 
}

void D1WaypointPublisher::NavThroughPosesFeedbackCallback(const GoalHandleNavigateNavigateThroughPoses::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback){
  //std::cout << "8" << std::endl;

  number_of_poses_remaining_ = feedback->number_of_poses_remaining;
  //RCLCPP_INFO(get_logger(), "number of poses remaining = %zu", (size_t)feedback->number_of_poses_remaining);
}

void D1WaypointPublisher::PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

  //geometry_msgs::msg::Pose _;
  find_point_ = 1;

  // 目標座標を取得
  //double goal_x = msg->pose.position.x;
  //double goal_y = msg->pose.position.y;

  goal_x = msg->pose.position.x;
  goal_y = msg->pose.position.y;

  // 現在位置との距離と角度を計算
  double delta_x = goal_x - current_pose_.position.x;
  double delta_y = goal_y - current_pose_.position.y;
  //double distance = 100;
  //distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
  double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

  double target_angle = std::atan2(delta_y, delta_x);

  // 目標座標に向かうためのTwistメッセージを作成
  auto twist_msg = geometry_msgs::msg::Twist();

  // 距離が十分に小さい場合、停止
  if (distance < 0.2)
  {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
      goal_point_ = 1;
  }
  else
  {
      // 前進速度と回転速度を設定
      // odomの座標系とcsvファイルの座標系が違う
      // ros2 topic echo /odom |grep position -4 でodomの座標系を確認
      twist_msg.linear.x = 0.3 * distance;
      twist_msg.angular.z = 0.3 * (target_angle - current_yaw_);
      std::cout << "twist_msg" << std::endl;
  }

  twist_pub_->publish(twist_msg);
}

void D1WaypointPublisher::AreaCallback(const std_msgs::msg::String::SharedPtr msg){
  find_character_ = 1;
  area_character_.data = msg->data;
}

void D1WaypointPublisher::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  //geometry_msgs::msg::Pose current_pose_;
  current_pose_ = msg->pose.pose;
  tf2::Quaternion q(
      current_pose_.orientation.x,
      current_pose_.orientation.y,
      current_pose_.orientation.z,
      current_pose_.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_yaw_);
}