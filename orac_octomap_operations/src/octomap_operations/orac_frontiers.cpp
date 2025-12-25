#include "octomap_operations/octomap_ops.hpp"
using namespace std;
using namespace octomap;

/**
 * Extracts frontier voxels from the octomap.
 *
 * @param frontier Output vector to store the frontier voxel keys.
 * @param ot Shared pointer to the octomap.
 */
void octomap_ops_cls::extract_frontier(
  std::vector<octomap::OcTreeKey>& frontier,std::shared_ptr<octomap::OcTree>& ot)
{
  frontier.clear();

  for (auto it = ot->begin_leafs(); it != ot->end_leafs(); ++it)
  {
    if (it->getOccupancy() > 0.5) continue;

    octomap::OcTreeKey k = it.getKey();
    bool is_frontier = false;

    for (int dx=-1; dx<=1 && !is_frontier; ++dx)
      for (int dy=-1; dy<=1 && !is_frontier; ++dy)
        for (int dz=-1; dz<=1 && !is_frontier; ++dz)
        {
          octomap::OcTreeKey nk(
            k.k[0]+dx, k.k[1]+dy, k.k[2]+dz);
          if (!ot->search(nk))
          {
            is_frontier = true;
            break;
          }
        }

    if (is_frontier)
      frontier.push_back(k);
  }
}

/**
 * Computes visible frontier voxels from a given pose.
 *
 * @param pose_vec Pose vector [x,y,z,w,qx,qy,qz].
 * @param key2idx Unordered map mapping voxel key hashes to frontier indices.
 * @param out_indices Output vector of visible frontier indices.
 * @param ot Octomap to query for ray casting.
 */
void octomap_ops_cls::compute_visible_frontier(
  std::vector<float>& pose_vec,
  std::unordered_map<uint64_t,int>& key2idx,
  std::vector<int>& out_indices, std::shared_ptr<octomap::OcTree>& ot)
{
  out_indices.clear();

  // Extract position and quaternion
  Eigen::Vector3d camera_pos(pose_vec[0], pose_vec[1], pose_vec[2]);
  Eigen::Quaterniond q(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
  q.normalize();

  double r_ray_cast=params_.r_max;
  int total_rays = 0;
  int total_keys_checked = 0;
  octomap::KeyRay ray;
  for (int i = 0; i < static_cast<int>(unit_rays_normal_view.size()); ++i)
    {
      const auto& u = unit_rays_normal_view[i];
      Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
      
      // Transform ray direction by rotation
      Eigen::Vector3d rotated_ray = q * ray_vec;
      Eigen::Vector3d end_point_world = camera_pos + rotated_ray;
      
      octomap::point3d origin(camera_pos[0], camera_pos[1], camera_pos[2]);
      octomap::point3d end_point(end_point_world[0], end_point_world[1], end_point_world[2]);
      octomap::point3d direction(end_point_world[0]-camera_pos[0], end_point_world[1]-camera_pos[1], end_point_world[2]-camera_pos[2]);
      direction.normalize();
      double max_range=params_.r_max;
      bool found_endpoint = ot->castRay( origin, direction, end_point, true, max_range ); // ignore unknown cells
      ray.reset();
      ot->computeRayKeys(origin, end_point, ray); 

      for (auto& k : ray)
      {
        total_keys_checked++;
        
        // Check the voxel hit by the ray
        uint64_t id = (uint64_t(k.k[0]) << 42) | (uint64_t(k.k[1]) << 21) | uint64_t(k.k[2]);
        auto it = key2idx.find(id);
        if (it != key2idx.end())
          out_indices.push_back(it->second);
        
        // Expand to neighboring voxels to compensate for sparse ray casting
        // expansion_radius determines how far to search (0=none, 1=26 neighbors, 2=124 neighbors, etc.)
        int expansion_radius = params_.frontier_neighbor_expansion;
        if (expansion_radius > 0)
        {
          for (int dx = -expansion_radius; dx <= expansion_radius; ++dx)
          {
            for (int dy = -expansion_radius; dy <= expansion_radius; ++dy)
            {
              for (int dz = -expansion_radius; dz <= expansion_radius; ++dz)
              {
                if (dx == 0 && dy == 0 && dz == 0) continue; // Skip the center voxel (already checked)
                
                octomap::OcTreeKey neighbor_key(k.k[0] + dx, k.k[1] + dy, k.k[2] + dz);
                uint64_t neighbor_id = (uint64_t(neighbor_key.k[0]) << 42) | 
                                       (uint64_t(neighbor_key.k[1]) << 21) | 
                                       uint64_t(neighbor_key.k[2]);
                
                auto neighbor_it = key2idx.find(neighbor_id);
                if (neighbor_it != key2idx.end())
                  out_indices.push_back(neighbor_it->second);
              }
            }
          }
        }
      }
  }
  
  ROS_DEBUG_STREAM("Rays cast: " << total_rays << ", Keys checked: " << total_keys_checked << ", Matches found: " << out_indices.size());
  
  sort(out_indices.begin(), out_indices.end());
  out_indices.erase(
    unique(out_indices.begin(), out_indices.end()),
    out_indices.end());
}

/**
 * Parallel version of compute_visible_frontier using OpenMP.
 * Computes visible frontier voxels from a given pose and accumulates results.
 *
 * @param pose_vec Pose vector [x,y,z,w,qx,qy,qz].
 * @param key2idx Unordered map mapping voxel key hashes to frontier indices.
 * @param out_indices Output vector of visible frontier indices.
 * @param ot Octomap to query for ray casting.
 */
void octomap_ops_cls::compute_visible_frontier_parallel(
  std::vector<float>& pose_vec,
  std::unordered_map<uint64_t,int>& key2idx,
  std::vector<int>& out_indices, std::shared_ptr<octomap::OcTree>& ot)
{
  out_indices.clear();
  
  // Extract position and quaternion
  Eigen::Vector3d camera_pos(pose_vec[0], pose_vec[1], pose_vec[2]);
  Eigen::Quaterniond q(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
  q.normalize();

  double fov_y = params_.hfov, fov_p = params_.vfov;
  double dphi = params_.dphi, dtheta = params_.dtheta;
  double r_ray_cast=params_.r_max;

  // Calculate number of iterations for each loop
  int n_theta = (int)ceil(fov_y / dtheta);
  int n_phi = (int)ceil(fov_p / dphi);

  int total_rays = n_theta * n_phi;
  int total_keys_checked = 0;
  int expansion_radius = params_.frontier_neighbor_expansion;

  // Parallel region with thread-local accumulation
  #pragma omp parallel
  {
    std::vector<int> thread_indices;  // Thread-local indices accumulation
    octomap::KeyRay ray;              // Thread-local: each thread gets its own instance
    int thread_keys_checked = 0;

    #pragma omp for schedule(guided, 4)
    for (int i = 0; i < static_cast<int>(unit_rays_normal_view.size()); ++i)
    {
        const auto& u = unit_rays_normal_view[i];
        Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
        
        // Transform ray direction by rotation
        Eigen::Vector3d rotated_ray = q * ray_vec;
        Eigen::Vector3d end_point_world = camera_pos + rotated_ray;

        octomap::point3d origin(camera_pos[0], camera_pos[1], camera_pos[2]);
        octomap::point3d end_point(end_point_world[0], end_point_world[1], end_point_world[2]);
        octomap::point3d direction(end_point_world[0]-camera_pos[0], end_point_world[1]-camera_pos[1], end_point_world[2]-camera_pos[2]);
        direction.normalize();
        double max_range=params_.r_max;
        bool found_endpoint = ot->castRay( origin, direction, end_point, true, max_range ); // ignore unknown cells
        
        ray.reset();
        ot->computeRayKeys(origin, end_point, ray); 

        for (auto& k : ray)
        {
          thread_keys_checked++;
          
          // Check the voxel hit by the ray
          uint64_t id = (uint64_t(k.k[0]) << 42) | (uint64_t(k.k[1]) << 21) | uint64_t(k.k[2]);
          auto it = key2idx.find(id);
          if (it != key2idx.end())
            thread_indices.push_back(it->second);
          
          // Expand to neighboring voxels to compensate for sparse ray casting
          if (expansion_radius > 0)
          {
            for (int dx = -expansion_radius; dx <= expansion_radius; ++dx)
            {
              for (int dy = -expansion_radius; dy <= expansion_radius; ++dy)
              {
                for (int dz = -expansion_radius; dz <= expansion_radius; ++dz)
                {
                  if (dx == 0 && dy == 0 && dz == 0) continue; // Skip the center voxel (already checked)
                  
                  octomap::OcTreeKey neighbor_key(k.k[0] + dx, k.k[1] + dy, k.k[2] + dz);
                  uint64_t neighbor_id = (uint64_t(neighbor_key.k[0]) << 42) | 
                                         (uint64_t(neighbor_key.k[1]) << 21) | 
                                         uint64_t(neighbor_key.k[2]);
                  
                  auto neighbor_it = key2idx.find(neighbor_id);
                  if (neighbor_it != key2idx.end())
                    thread_indices.push_back(neighbor_it->second);
                }
              }
            }
          }
        }
    }

    // Merge thread-local results into global output
    #pragma omp critical
    {
      out_indices.insert(out_indices.end(), thread_indices.begin(), thread_indices.end());
      total_keys_checked += thread_keys_checked;
    }
  }
  
  ROS_DEBUG_STREAM("Rays cast: " << total_rays << ", Keys checked: " << total_keys_checked << ", Matches found: " << out_indices.size());
  
  // Remove duplicates from all threads' contributions
  sort(out_indices.begin(), out_indices.end());
  out_indices.erase(
    unique(out_indices.begin(), out_indices.end()),
    out_indices.end());
}
