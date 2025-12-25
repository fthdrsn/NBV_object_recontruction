#ifndef READ_PARAMS_H
#define READ_PARAMS_H


struct Params
{
  double hfov;
  double vfov;

  double hfov_expanded;
  double vfov_expanded;

  double dphi;
  double dtheta;
  double r_max;
  bool raycast_use_low_res_octomap;
  std::string octomap_topic_name;
  std::string lowres_octomap_topic_name;  // Optional: coarse octomap topic
  std::string octomap_save_path;

  double cell_size;

  double focus_point_r;
  
  int frontier_neighbor_expansion;  // Radius for neighbor expansion (0=no expansion, 1=26 neighbors, 2=124 neighbors)
  
  std::vector<double> boundary_min;
  std::vector<double> boundary_max;

  bool use_low_res;
  double low_res_cell_size;

};

Params readParams();


#endif