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

  std::string octomap_topic_name;
  std::string octomap_save_path;

  double cell_size;

  double focus_point_r;
  
  std::vector<double> boundary_min;
  std::vector<double> boundary_max;

};

Params readParams();


#endif