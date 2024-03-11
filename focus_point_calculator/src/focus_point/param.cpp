#include <ros/ros.h>

#include <focus_point/param.h>


Params readParams()
{
  Params params;
  std::string ns = ros::this_node::getNamespace();
  
  params.hfov = 74;
  if (!ros::param::get(ns + "/camera/horizontal_fov", params.hfov)) {
    ROS_WARN_STREAM("No horizontal fov specified. Default: " << params.hfov);
  }

  params.vfov = 60;
  if (!ros::param::get(ns + "/camera/vertical_fov", params.vfov)) {
    ROS_WARN_STREAM("No vertical fov specified. Default: " << params.vfov);
  }
  
  params.hfov_expanded = 90;
  if (!ros::param::get(ns + "/camera/horizontal_fov_expanded", params.hfov_expanded)) {
    ROS_WARN_STREAM("No horizontal fov specified. Default: " << params.hfov_expanded);
  }

  params.vfov_expanded = 90;
  if (!ros::param::get(ns + "/camera/vertical_fov_expanded", params.vfov_expanded)) {
    ROS_WARN_STREAM("No vertical fov specified. Default: " << params.vfov_expanded);
  }


  params.dphi = 5;
  if (!ros::param::get(ns + "/raycast/dphi", params.dphi)) {
    ROS_WARN_STREAM("No dphi specified. Default: " << params.dphi);
  }

  params.dtheta = 5;
  if (!ros::param::get(ns + "/raycast/dtheta", params.dtheta)) {
    ROS_WARN_STREAM("No dtheta specified. Default: " << params.dtheta);
  }
 
  params.r_max = 5.0;
  if (!ros::param::get(ns + "/raycast/r_max", params.r_max)) {
    ROS_WARN_STREAM("No /raycast/r_max specified. Default: " << params.r_max);
  }

  
  params.boundary_min={-2.75, -2.75, 0.1};
  if (!ros::param::get(ns + "/boundary/min", params.boundary_min)) {
    ROS_WARN_STREAM("No /boundary/min specified. Using Default" );
  }

  params.boundary_max={2.75, 2.75, 5};
  if (!ros::param::get(ns + "/boundary/max", params.boundary_max)) {
    ROS_WARN_STREAM("No /boundary/max specified. Using Default" );
  }

  params.octomap_topic_name = "/octomap_full";
  if (!ros::param::get(ns + "/octomap/topic_name", params.octomap_topic_name )) {
    ROS_WARN_STREAM("No /octomap/topic_name specified. Default: " << params.octomap_topic_name);
  }
  params.octomap_save_path = "/home/fth/NBV_Youbot/src/map_";
  if (!ros::param::get(ns + "/octomap/save_path", params.octomap_save_path)) {
    ROS_WARN_STREAM("No /octomap/save_path specified. Default: " << params.octomap_save_path);
  }

  params.focus_point_r = 2.5;
  if (!ros::param::get(ns + "/focus_pnt/focus_r", params.focus_point_r)) {
    ROS_WARN_STREAM("No /focus_pnt/focus_r specified. Default: " << params.focus_point_r);
  }
 
  params.cell_size = 0.03;
  if (!ros::param::get(ns + "/octomap/cell_size", params.cell_size)) {
    ROS_WARN_STREAM("No /octomap/cell_size specified. Default: " << params.cell_size);
  }
  
  return params;
  
}