cmake_minimum_required(VERSION 3.0.2)
project(orac_reconstruction_services)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  FILES
  coverage_srv.srv
  focus_point_srv.srv
  get_frontier_srv.srv
  get_view_frontier_srv.srv
  save_octomap_srv.srv
  stop_octomap_update_srv.srv
  view_evaluate_srv.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
