#include "ros/ros.h"
#include "math.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>
#include <dqrobotics/DQ.h>
#include "focus_point_calculator/coverage_srv.h"
#include "focus_point_calculator/focus_point_srv.h"
#include "focus_point_calculator/view_evaluate_srv.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <focus_point/param.h>

class focus_point_cls
{
 public:
   ros::NodeHandle rosNode_;
   focus_point_cls(const ros::NodeHandle &rosNode);
   void octomap_callback(const octomap_msgs::Octomap& msg);
   bool is_inside_boundaries(Eigen::Vector4d point);
   bool calculate_ocluded_volume(focus_point_calculator::coverage_srv::Request &req, focus_point_calculator::coverage_srv::Response &resp);
   Eigen::Vector3d get_focus_point(std::vector<float> pose_vec);
   double get_view_ig_ent(std::vector<float> pose_vec);
   double get_view_ig_rsv(std::vector<float> pose_vec);

   bool calculate_focus_point(focus_point_calculator::focus_point_srv::Request &req,focus_point_calculator::focus_point_srv::Response &resp);
   bool calculate_view_igs(focus_point_calculator::view_evaluate_srv::Request &req,focus_point_calculator::view_evaluate_srv::Response &resp);
   void save_tree();
   bool is_camera_initiated;
   bool is_octomap_received;
   ros::Subscriber octo_map_sub;
   ros::ServiceServer coverage_service;
   ros::ServiceServer focus_point_service;
   ros::ServiceServer view_evaluate_service;
   std::shared_ptr<octomap::OcTree> ot_;
   std::vector<double> boundary_max;
   std::vector<double> boundary_min;
   
   Params params_;

   int counter;

  std::ofstream file_to_save;
  std::ofstream file_to_save_pose;
  int ctr;
  std::stringstream filename;

};
