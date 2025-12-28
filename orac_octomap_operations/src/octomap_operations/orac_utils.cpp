#include "octomap_operations/octomap_ops.hpp"
using namespace std;
using namespace octomap;

/**
 * Calculates a set of unit rays based on specified field of view and angular increments w.r.t camera frame.
 * 
 *
 * @param unit_ray_set Output vector to store the calculated unit rays.
 * @param hfov Horizontal field of view in degrees.
 * @param vfov Vertical field of view in degrees.
 * @param dphi Angular increment in vertical direction in degrees.
 * @param dtheta Angular increment in horizontal direction in degrees.
 *       y  
        |    assumed camera frame: z-forward, x-left, y-up
  x_____|  
  */
void octomap_ops_cls::calculate_unit_ray_set(std::vector<Eigen::Vector3d>& unit_ray_set, double hfov, double vfov, double dphi, double dtheta)
{ 

  unit_ray_set.clear();
  int n_theta = (int)ceil(hfov / dtheta);
  int n_phi = (int)ceil(vfov / dphi);

  for (int i_theta = 0; i_theta < n_theta; ++i_theta)
  {
    for (int i_phi = 0; i_phi < n_phi; ++i_phi)
    {
      double theta = -hfov/2.0 + i_theta * dtheta;
      double phi = -vfov/2.0 + i_phi * dphi;
      
      double theta_rad = M_PI * theta / 180.0f;
      double sin_theta = sin(theta_rad), cos_theta = cos(theta_rad);
      
      double phi_rad = M_PI * phi / 180.0f;
      double sin_phi = sin(phi_rad), cos_phi = cos(phi_rad);
      
      Eigen::Vector3d vec;
      vec[0] = sin_theta * cos_phi;
      vec[1] = -cos_theta * sin_phi;
      vec[2] = cos_theta * cos_phi;
      
      unit_ray_set.push_back(vec);
    }
  }
}

/**
 * Check if the given point inside the boundaries
 *
 * @param point 3D point to check if it is inside boundaries.
 * @return True is the point is inside the boundaries.
 */
bool octomap_ops_cls::is_inside_boundaries(Eigen::Vector3d point)
{
  return point[0] > boundary_min[0] and point[0] < boundary_max[0] and
         point[1] > boundary_min[1] and point[1] < boundary_max[1] and
         point[2] > boundary_min[2] and point[2] < boundary_max[2];
}
