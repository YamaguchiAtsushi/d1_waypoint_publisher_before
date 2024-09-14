#include "d1_waypoint_manager/d1_waypoint_manager.hpp"  // include local header

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D1WaypointPublisher>());
  rclcpp::shutdown();

  return 0;
}

