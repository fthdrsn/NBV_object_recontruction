#include "octomap_operations/octomap_ops.hpp"
using namespace std;
using namespace octomap;


/**
 * Calculates the focus point from the given view. Focus point is the point along the ray with maximum entropy.
 *
 * @param pose_vec Rays are calculated  using this pose vector [x,y,z,w,qx,qy,qz] and expanded FoV.
 * @return Focus point for the given view (pose)
 */
Eigen::Vector3d octomap_ops_cls::get_focus_point(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot)
{
  // Extract position and quaternion
  Eigen::Vector3d camera_pos(pose_vec[0], pose_vec[1], pose_vec[2]);
  Eigen::Quaterniond q(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
  q.normalize();

  double r_ray_cast=params_.r_max;
  float r_max_last=params_.focus_point_r;
  
  double best_ray_entropy=-1e100;
  Eigen::Vector3d focus_point(0,0,0);
  octomap::KeyRay ray;
  for (int i = 0; i < static_cast<int>(unit_rays_expanded_view.size()); ++i)
    {
      const auto& u = unit_rays_expanded_view[i];
      Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
      
      // Transform ray direction by rotation and apply to endpoint calculation
      Eigen::Vector3d rotated_ray = q * ray_vec;
      Eigen::Vector3d end_point_world = camera_pos + rotated_ray;

      octomap::point3d origin(camera_pos[0], camera_pos[1], camera_pos[2]);
      octomap::point3d end_point(end_point_world[0], end_point_world[1], end_point_world[2]);
      octomap::point3d direction(end_point_world[0]-camera_pos[0], end_point_world[1]-camera_pos[1], end_point_world[2]-camera_pos[2]);
      direction.normalize();
      double max_range=params_.r_max;
      bool found_endpoint = ot->castRay( origin, direction, end_point, true, max_range ); // ignore unknown cells
      ray.reset();
      ot->computeRayKeys(origin, end_point, ray); //Compute octree keys for all traversed voxels (excluding the end_point)
      //Calculate the entropy of the ray
      double ray_entropy(0);
      for( KeyRay::iterator it = ray.begin() ; it!=ray.end(); ++it )
        {
        octomap::point3d coord = ot->keyToCoord(*it);
        Eigen::Vector4d v(coord.x(), coord.y(), coord.z(), 0);
        if (!is_inside_boundaries(v))
              continue;

        octomap::OcTreeNode* node =ot->search(*it);
        double prob_occupancy(0);
        if(!node){
          //Node is unknown
          prob_occupancy=0.5;
        }else{
          //Node is free or occupied, get probability
          prob_occupancy=node->getOccupancy();
        }
        ray_entropy+=entropy(prob_occupancy);
        }

        //Finally, include th entropy of the end point
        // Check the end point
        OcTreeKey end_key;
        if(ot->coordToKeyChecked(end_point,end_key))
        {
          octomap::OcTreeNode* end_node =ot->search(end_key);
          double prob_occupancy(0);
          if(!end_node){
            //Node is unknown
            prob_occupancy=0.5;
          }else{
            //Node is free or occupied, get probability
            prob_occupancy=end_node->getOccupancy();
          }

          ray_entropy+=entropy(prob_occupancy);
        }     
        
        if(ray_entropy>best_ray_entropy)
        {
          best_ray_entropy=ray_entropy;
          
          // Transform the focus point (at distance r_max_last along the ray direction)
          Eigen::Vector3d focus_point_local(r_max_last * u.x(), r_max_last * u.y(), r_max_last * u.z());
          Eigen::Vector3d focus_point_rotated = q.cast<double>() * focus_point_local;
          focus_point = camera_pos + focus_point_rotated;
        }

    }
      
    return focus_point;

}

/**
 * Calculates the focus point from the given view (parallel version using OpenMP). Focus point is the point along the ray with maximum entropy.
 * 
 * @param pose_vec Rays are calculated  using this pose vector [x,y,z,w,qx,qy,qz] and expanded FoV.
 * @return Focus point for the given view (pose)
 */

Eigen::Vector3d octomap_ops_cls::get_focus_point_parallel(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot)
{
  // Extract position and quaternion
  Eigen::Vector3d camera_pos(pose_vec[0], pose_vec[1], pose_vec[2]);
  Eigen::Quaterniond q(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
  q.normalize();

  double r_ray_cast=params_.r_max;
  float r_max_last=params_.focus_point_r;

  double best_ray_entropy = -1e100;
  Eigen::Vector3d best_focus_point(0,0,0);

  #pragma omp parallel
  {
    double thread_best_entropy = -1e100;
    Eigen::Vector3d thread_focus_point(0,0,0);
    octomap::KeyRay ray;  // Thread-local: each thread gets its own instance

    #pragma omp for schedule(static)
    for (int i = 0; i < static_cast<int>(unit_rays_expanded_view.size()); ++i)
    {
        const auto& u = unit_rays_expanded_view[i];
        Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
        
        // Transform ray direction by rotation
        Eigen::Vector3d rotated_ray = q * ray_vec;
        Eigen::Vector3d end_point_world = camera_pos + rotated_ray;
       
        octomap::point3d origin(camera_pos[0], camera_pos[1], camera_pos[2]);
        octomap::point3d end_point(end_point_world[0], end_point_world[1], end_point_world[2]);
        octomap::point3d direction(end_point_world[0]-camera_pos[0], end_point_world[1]-camera_pos[1], end_point_world[2]-camera_pos[2]);
        direction.normalize();
        double max_range=params_.r_max;
        bool found_endpoint = ot->castRay( origin, direction, end_point, true, max_range );

        // Compute ray entropy
        ray.reset();
        ot->computeRayKeys(origin, end_point, ray);
        double ray_entropy = 0.0;
        for( octomap::KeyRay::iterator it = ray.begin() ; it!=ray.end(); ++it )
        {
          octomap::point3d coord = ot->keyToCoord(*it);
          Eigen::Vector4d v(coord.x(), coord.y(), coord.z(), 0);
          if (!is_inside_boundaries(v))
               continue;

          octomap::OcTreeNode* node =ot->search(*it);
          double prob_occupancy(0);
          if(!node){
            prob_occupancy=0.5;
          }else{
            prob_occupancy=node->getOccupancy();
          }
          ray_entropy += entropy(prob_occupancy);
        }

        // include entropy of the end point
        octomap::OcTreeKey end_key;
        if(ot->coordToKeyChecked(end_point,end_key))
        {
          octomap::OcTreeNode* end_node =ot->search(end_key);
          double prob_occupancy(0);
          if(!end_node){
            prob_occupancy=0.5;
          }else{
            prob_occupancy=end_node->getOccupancy();
          }
          ray_entropy += entropy(prob_occupancy);
        }

        if (ray_entropy > thread_best_entropy)
        {
          thread_best_entropy = ray_entropy;
          
          // Transform the focus point (at distance r_max_last along the ray direction)
          Eigen::Vector3d focus_point_local(r_max_last * u.x(), r_max_last * u.y(), r_max_last * u.z());
          Eigen::Vector3d focus_point_rotated = q * focus_point_local;
          thread_focus_point = camera_pos + focus_point_rotated;
        }
      }

    // Combine thread local best into global best
    #pragma omp critical
    {
      if (thread_best_entropy > best_ray_entropy)
      {
        best_ray_entropy = thread_best_entropy;
        best_focus_point = thread_focus_point;
      }
    }
  }

  return best_focus_point;
}
