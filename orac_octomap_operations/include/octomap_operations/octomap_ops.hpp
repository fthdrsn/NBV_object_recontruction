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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include "orac_reconstruction_services/coverage_srv.h"
#include "orac_reconstruction_services/focus_point_srv.h"
#include "orac_reconstruction_services/view_evaluate_srv.h"
#include "orac_reconstruction_services/save_octomap_srv.h"
#include "orac_reconstruction_services/get_frontier_srv.h"
#include "orac_reconstruction_services/get_view_frontier_srv.h"
#include "orac_reconstruction_services/stop_octomap_update_srv.h"
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <chrono>
#include <omp.h>
#include <octomap_operations/param.h>
#include <mutex>
class octomap_ops_cls
{
 public:
  ros::NodeHandle rosNode_;
  octomap_ops_cls(const ros::NodeHandle &rosNode);
  void octomap_callback(const octomap_msgs::Octomap& msg);
  void octomap_coarse_callback(const octomap_msgs::Octomap& msg);

  bool is_inside_boundaries(Eigen::Vector3d point);
  bool calculate_occluded_volume(orac_reconstruction_services::coverage_srv::Request &req, orac_reconstruction_services::coverage_srv::Response &resp);
  Eigen::Vector3d get_focus_point(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot);
  Eigen::Vector3d get_focus_point_parallel(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot);
  double get_view_ig_ent(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot);
  double get_view_ig_ent_parallel(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot);
  double get_view_ig_rsv(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot);
  double get_view_ig_rsv_parallel(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot);
  void extract_frontier(std::vector<octomap::OcTreeKey>& frontier,std::shared_ptr<octomap::OcTree>& ot);
  bool calculate_focus_point(orac_reconstruction_services::focus_point_srv::Request &req,orac_reconstruction_services::focus_point_srv::Response &resp);
  bool calculate_view_igs(orac_reconstruction_services::view_evaluate_srv::Request &req,orac_reconstruction_services::view_evaluate_srv::Response &resp);
  bool save_octomap_to_path(orac_reconstruction_services::save_octomap_srv::Request &req,orac_reconstruction_services::save_octomap_srv::Response &resp);
  bool calculate_frontier(orac_reconstruction_services::get_frontier_srv::Request &req, orac_reconstruction_services::get_frontier_srv::Response &resp);
  bool calculate_view_frontier(orac_reconstruction_services::get_view_frontier_srv::Request &req, orac_reconstruction_services::get_view_frontier_srv::Response &resp);
  bool stop_octomap_update(orac_reconstruction_services::stop_octomap_update_srv::Request &req, orac_reconstruction_services::stop_octomap_update_srv::Response &resp);
  void compute_visible_frontier(std::vector<float>& pose_vec, std::unordered_map<uint64_t,int>& key2idx,
  std::vector<int>& out_indices,std::shared_ptr<octomap::OcTree>& ot);
  void compute_visible_frontier_parallel(std::vector<float>& pose_vec, std::unordered_map<uint64_t,int>& key2idx,
    std::vector<int>& out_indices,std::shared_ptr<octomap::OcTree>& ot);
  std::vector<octomap::OcTreeKey> frontier_voxels;
  void save_tree();
  double entropy(double p);
  void calculate_unit_ray_set(std::vector<Eigen::Vector3d>& unit_ray_set,double hfov, double vfov, double dphi, double dtheta);
  std::vector<Eigen::Vector3d> unit_rays_normal_view;
  std::vector<Eigen::Vector3d> unit_rays_expanded_view;

  ros::Subscriber octo_map_sub;
  ros::ServiceServer coverage_service;
  ros::ServiceServer focus_point_service;
  ros::ServiceServer view_evaluate_service;
  ros::ServiceServer save_octomap_service;
  ros::ServiceServer get_frontier_service;
  ros::ServiceServer get_view_frontier_service;
  ros::ServiceServer stop_octomap_update_service;
  std::shared_ptr<octomap::OcTree> ot_;
  std::shared_ptr<octomap::OcTree> ot_coarse_;
  std::atomic<bool> is_octomap_received{false};
  std::atomic<bool> is_coarse_octomap_received{false};
  std::atomic<bool> block_octomap_updates{false};
  ros::Subscriber octo_map_coarse_sub;
  std::mutex octomap_mutex;
  std::mutex coarse_octomap_mutex;
  std::vector<double> boundary_max;
  std::vector<double> boundary_min;
  Params params_;
  int counter;

  std::ofstream file_to_save;
  std::ofstream file_to_save_pose;
  int ctr;
  std::stringstream filename;

};