#include "focus_point/focus_point.hpp"
using namespace std;
using namespace octomap;
using namespace DQ_robotics;

focus_point_cls::focus_point_cls(const ros::NodeHandle &rosNode):rosNode_(rosNode)
{
  params_ = readParams();
  boundary_max=params_.boundary_max;
  boundary_min=params_.boundary_min;
  octo_map_sub =rosNode_.subscribe(params_.octomap_topic_name, 1, &focus_point_cls::octomap_callback, this);
  coverage_service=rosNode_.advertiseService("get_coverage",&focus_point_cls::calculate_occluded_volume,this);
  focus_point_service=rosNode_.advertiseService("get_focus_point",&focus_point_cls::calculate_focus_point,this);
  view_evaluate_service=rosNode_.advertiseService("get_view_igs",&focus_point_cls::calculate_view_igs,this);
  save_octomap_service=rosNode_.advertiseService("save_octomap",&focus_point_cls::save_octomap_to_path,this);
  get_frontier_service=rosNode_.advertiseService("get_frontier",&focus_point_cls::calculate_frontier,this);
  get_view_frontier_service=rosNode_.advertiseService("get_view_frontier",&focus_point_cls::calculate_view_frontier,this);
  stop_octomap_update_service=rosNode_.advertiseService("stop_octomap_update",&focus_point_cls::stop_octomap_update,this);
  is_octomap_received=false;
  counter=0;
  calculate_unit_ray_set(unit_rays_normal_view,params_.hfov,params_.vfov,params_.dphi,params_.dtheta);
  calculate_unit_ray_set(unit_rays_expanded_view,params_.hfov_expanded,params_.vfov_expanded,params_.dphi,params_.dtheta);
  
}

/**
 * Calculates the entropy given a probability value (probability of being occupied).
 *
 * @param p Probability value (between 0 and 1).
 * @return Calculated entropy.
 */
inline double focus_point_cls::entropy(double p)
{
    constexpr double eps = 1e-6;
    p = std::min(1.0 - eps, std::max(eps, p));
    return -p * log(p) - (1.0 - p) * log(1.0 - p);
}

/**
 * This function is called when octomap data is published.
 *
 * @param msg Octomap message.
 */
void focus_point_cls::octomap_callback(const octomap_msgs::Octomap& msg)
{
  // Check if updates are blocked
  // It can be blocked via service call "stop_octomap_update" to make sure the octomap does not change during the calculation
  if (block_octomap_updates)
  {
    ROS_DEBUG_STREAM("Octomap update blocked by user");
    return;
  }

  ROS_DEBUG_STREAM("Received octomap message");
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = dynamic_cast<octomap::OcTree*>(aot);
  if (!ot)
  {
      ROS_ERROR("Received Octomap is not an OcTree");
      delete aot;
      return;
  }
  {
    std::lock_guard<std::mutex> lock(octomap_mutex);
    ot_ = std::make_shared<octomap::OcTree>(*ot);
  }
  delete ot;
  is_octomap_received=true;
  lowres_needs_rebuild = true;  // Update low-res tree when only necessary
  ROS_DEBUG_STREAM("Octomap updated");
}

/**
 * Service function to stop or resume octomap updates.
 *
 * @param req Service request.
 * @param resp Service response.
 */ 
bool focus_point_cls::stop_octomap_update(focus_point_calculator::stop_octomap_update_srv::Request &req, focus_point_calculator::stop_octomap_update_srv::Response &resp)
{
  block_octomap_updates=req.block_updates;
  if(block_octomap_updates)
    ROS_INFO("Octomap updates have been blocked.");
  else
    ROS_INFO("Octomap updates have been unblocked.");
  resp.success=true;
  return true;
}
/**
 * Rebuilds the low-resolution octomap by aggregating probabilities from the high-resolution octomap.
 */
void focus_point_cls::rebuild_lowres_tree()
{
  // Build a lower-resolution tree by aggregating fine voxels' probabilities
  double new_res = params_.low_res_cell_size; 
  try
  {
    std::shared_ptr<octomap::OcTree> coarse_model = std::make_shared<octomap::OcTree>(new_res);

    for (octomap::OcTree::leaf_iterator it = ot_->begin_leafs(); it != ot_->end_leafs(); ++it)
    {
      octomap::point3d coord = it.getCoordinate();
      float log_odds = it->getLogOdds();
      coarse_model->setNodeValue(coord, log_odds);
    }

    coarse_model->updateInnerOccupancy();
    coarse_model->toMaxLikelihood();
    coarse_model->prune();
    ot_lowres_ = coarse_model;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Failed to rebuild low-res octomap: " << e.what());
    ot_lowres_.reset();
  }
}

/**
 * Service function to calculate the information gain (IG) for a list of views.
 * @param req Service request containing the list of views and IG method.
 * @param resp Service response containing the calculated IGs for each view.
 */
 bool focus_point_cls::calculate_view_igs(focus_point_calculator::view_evaluate_srv::Request &req,focus_point_calculator::view_evaluate_srv::Response &resp)
 {
  bool use_low_res=params_.use_low_res;
  if(is_octomap_received)
  {
  std::shared_ptr<octomap::OcTree> ot;
  {
        std::lock_guard<std::mutex> lock(octomap_mutex);
        if (use_low_res)  
        {
          // Lazy rebuild: only rebuild if stale
          if (lowres_needs_rebuild)
          {
            auto map_copy_time = std::chrono::steady_clock::now();
            rebuild_lowres_tree();
            lowres_needs_rebuild = false;
            auto map_copy_end_time = std::chrono::steady_clock::now();
            ROS_INFO_STREAM("Low-res octomap rebuild time: "
                            << std::chrono::duration_cast<std::chrono::milliseconds>(map_copy_end_time - map_copy_time).count()
                            << " ms");
          }
          ot = ot_lowres_;
        }
        else
          ot = ot_;
  }
    if (req.view_list.size() % 7 != 0)
    {
        ROS_ERROR("view_list size is not a multiple of 7");
        return false;
    }

    const bool use_rsv = (req.ig_method == "RSV");
    const bool use_ent = (req.ig_method == "ENT");

    if (!use_rsv && !use_ent)
    {
        ROS_ERROR("Invalid IG method: %s", req.ig_method.c_str());
        return false;
    }

    std::vector<float> resultVec;
    int num_poses = req.view_list.size() / 7;
    resultVec.resize(num_poses);

    bool parallel_enabled = req.run_parallel;
    auto begin_total = std::chrono::steady_clock::now();

    // Parallelize across poses
    // For larger number of poses, it is better to parallelize the views instead of rays
    #pragma omp parallel for if(parallel_enabled) schedule(guided, 4)
    for (int pose_idx = 0; pose_idx < num_poses; ++pose_idx)
    {
        std::vector<float> pose(req.view_list.begin() + pose_idx * 7,
                                req.view_list.begin() + pose_idx * 7 + 7);

         double ig = use_rsv
             ? get_view_ig_rsv(pose, ot)
             : get_view_ig_ent(pose, ot);
      

        resultVec[pose_idx] = ig;
    }  
     
    auto end_total = std::chrono::steady_clock::now();
    double totalTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(end_total - begin_total).count();

    ROS_INFO_STREAM("Total IG computation time: " << totalTimeMs << " ms for " 
                    << num_poses << " poses (avg: " << totalTimeMs / num_poses << " ms/pose)");

    resp.view_igs = std::move(resultVec);
    resp.elapsed_time = totalTimeMs; // Convert to seconds
    return true;
 }
 else
 {
 cout<<"Octomap is not received";
 resp.view_igs={};
 return false;
 
 }
  
 }


 
/**
 * Calculates total entropy of the given view.
 *
 * @param pose_vec Rays are calculated  using this pose vector [x,y,z,w,x,y,z].
 * @return Total entropy for the given view (pose)
 */
double focus_point_cls::get_view_ig_ent(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot)
{
    
  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;

 
  double r_ray_cast=params_.r_max;
  double view_entropy=0.0;
  Eigen::Vector4d end_point_dq;

   // Calculate number of iterations for each loop
  octomap::KeyRay ray;

  for (int i = 0; i < static_cast<int>(unit_rays_normal_view.size()); ++i)
    {
        const auto& u = unit_rays_normal_view[i];
        Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
        Eigen::Vector4d end_point_dq = vec4(translation(pose_dq*(1+E_*0.5*(ray_vec.x()*i_+ray_vec.y()*j_+ray_vec.z()*k_))));

        octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
        octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
        octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
        direction.normalize();
        double max_range=params_.r_max;
        bool found_endpoint = ot->castRay( origin, direction, end_point, true, max_range ); // ignore unknown cells
        //true if an occupied cell was hit, false if the maximum range or octree bounds are reached

        ray.reset();
        ot->computeRayKeys(origin, end_point, ray); //Compute octree keys for all traversed voxels (excluding the end_point)
        //Calculate the entropy of the ray
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
          view_entropy+=entropy(prob_occupancy);
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

          view_entropy+=entropy(prob_occupancy);
          
        }     


    }
    return view_entropy;

}

/**
 * Calculates the entropy of the given view (parallel version using OpenMP).
 *
 * @param pose_vec Rays are calculated  using this pose vector [x,y,z,w,x,y,z].
 * @return Total entropy for the given view (pose)
 */

double focus_point_cls::get_view_ig_ent_parallel(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot)
{
  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;

  double r_ray_cast=params_.r_max;
  double view_entropy=0.0;

  // Parallel region with collapse(2) for nested loops
  #pragma omp parallel
  {
    octomap::KeyRay ray;  // Thread-local: each thread gets its own instance
    #pragma omp for reduction(+:view_entropy) schedule(guided, 4)
    for (int i = 0; i < static_cast<int>(unit_rays_normal_view.size()); ++i)
    {
      const auto& u = unit_rays_normal_view[i];
      Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
      Eigen::Vector4d end_point_dq=vec4(translation(pose_dq*(1+E_*0.5*(ray_vec.x() *i_+ray_vec.y()*j_+ray_vec.z()*k_))));
    
      octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
      octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
      octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
      direction.normalize();
      double max_range=params_.r_max;
      bool found_endpoint = ot->castRay(origin, direction, end_point, true, max_range);

      ray.reset();
      ot->computeRayKeys(origin, end_point, ray);
      
      double ray_entropy_local = 0.0;
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
        ray_entropy_local+=entropy(prob_occupancy);
      }

      // Check the end point
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
        ray_entropy_local+=entropy(prob_occupancy);
      }
      
      view_entropy += ray_entropy_local;
      }
    } 
  
  return view_entropy;
}

/**
 * Calculates the count of rear-side voxels of the given view.
 *
 * @param pose_vec Rays are calculated  using this pose vector [x,y,z,w,x,y,z].
 * @return Count of rear-side voxels for the given view (pose)
 */


double focus_point_cls::get_view_ig_rsv(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot)
{

  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;

  double r_ray_cast=params_.r_max;
  double rear_side_voxel_count=0.0;

  Eigen::Vector4d end_point_dq;
  Eigen::Vector3d focus_point;
  octomap::KeyRay ray;
  for (int i = 0; i < static_cast<int>(unit_rays_normal_view.size()); ++i)
    {
      const auto& u = unit_rays_normal_view[i];
      Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
      end_point_dq = vec4(translation(pose_dq*(1+E_*0.5*(ray_vec.x()*i_+ray_vec.y()*j_+ray_vec.z()*k_))));

      octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
      octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
      octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
      direction.normalize();
      double max_range=params_.r_max;
      bool found_endpoint = ot->castRay(origin, direction, end_point, true, max_range); // ignore unknown cells
      //true if an occupied cell was hit, false if the maximum range or octree bounds are reached
      
      octomap::OcTreeNode* prev_node;
      if(found_endpoint) //The ray should hit an occupied voxel, otherwise there is no rear_side_voxel on this ray
      {
        ray.reset();
        ot->computeRayKeys(origin, end_point, ray); //Compute octree keys for all traversed voxels (excluding the end_point)
        if (ray.size()>1) //At least two voxels (the last one is occupied)
        {
          auto last_it = std::prev(ray.end());
          
          prev_node = ot->search(*last_it);
          //search the previous voxel of the end point
          // Increase rear side voxel count if the voxel before the occupied voxel is unknown
          // So each ray can have one rearside voxel
        if (!prev_node){
            rear_side_voxel_count++;
        }
        }
        
      }

    }
      
  return rear_side_voxel_count;
}

/**
 * Calculates the count of rear-side voxels of the given view (parallel version using OpenMP).
 *
 * @param pose_vec Rays are calculated  using this pose vector [x,y,z,w,x,y,z].
 * @return Count of rear-side voxels for the given view (pose)
 */

double focus_point_cls::get_view_ig_rsv_parallel(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot)
{
  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;

  double r_ray_cast=params_.r_max;
  double rear_side_voxel_count=0.0;



  #pragma omp parallel
  {
    octomap::KeyRay ray;  // Thread-local: each thread gets its own instance
    #pragma omp for reduction(+:rear_side_voxel_count) schedule(guided, 4)
  for (int i = 0; i < static_cast<int>(unit_rays_normal_view.size()); ++i)
    {
      const auto& u = unit_rays_normal_view[i];
      Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
      Eigen::Vector4d end_point_dq = vec4(translation(pose_dq*(1+E_*0.5*(ray_vec.x()*i_+ray_vec.y()*j_+ray_vec.z()*k_))));

      octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
      octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
      octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
      direction.normalize();
      double max_range=params_.r_max;
      
      bool found_endpoint = ot->castRay(origin, direction, end_point, true, max_range);

      
      if(found_endpoint)
      { 
        ray.reset();
        ot->computeRayKeys(origin, end_point, ray);
        if (ray.size()>1)
        {
          auto last_it = std::prev(ray.end());
          octomap::OcTreeNode* prev_node = ot->search(*last_it);
          if (!prev_node){
            rear_side_voxel_count++;
          }
        }
      }
  }
}
  
  return rear_side_voxel_count;
}
/**
 * It is service function which calculates the focus point.
 *
 * @param req Service request.
 * @param resp Service response.
 */

bool focus_point_cls::calculate_focus_point(focus_point_calculator::focus_point_srv::Request &req,focus_point_calculator::focus_point_srv::Response &resp)
{
  bool use_low_res=params_.use_low_res;
  if(is_octomap_received)
  {
  std::shared_ptr<octomap::OcTree> ot;
  {
        std::lock_guard<std::mutex> lock(octomap_mutex);
        
        if(use_low_res)
        {
          // Lazy rebuild: only rebuild if stale
          if (lowres_needs_rebuild)
          {
            rebuild_lowres_tree();
            lowres_needs_rebuild = false;
          }
          ot = ot_lowres_;
        }
        else
          ot = ot_;
  }
  std::cout<<"Start focus point calculation"<<std::endl;
  std::vector<float> req_vec=req.pose;

  std::vector<float> tmp_vec;
  std::vector<float> result_vec;
  Eigen::Vector3d focus_pnt;

  auto begin_focus_calculation= std::chrono::steady_clock::now();
  // Use parallel implementation when requested
  if (req.run_parallel)
    focus_pnt = get_focus_point_parallel(req_vec, ot);
  else
    focus_pnt = get_focus_point(req_vec, ot);
  auto end_focus_calculation= std::chrono::steady_clock::now();
  double totalTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(end_focus_calculation - begin_focus_calculation).count();
  std::cout<<"Focus point calculation time: "<< totalTimeMs << " ms"<<std::endl;
  resp.elapsed_time = totalTimeMs;
  result_vec.push_back(focus_pnt[0]);
  result_vec.push_back(focus_pnt[1]);
  result_vec.push_back(focus_pnt[2]);
  resp.focus_pnt=result_vec;
  counter++;
 
 }else
 {
 resp.focus_pnt={};
 }
  
  return true;
}
/**
 * Calculates the focus point from the given view. Focus point is the point along the ray with maximum entropy.
 *
 * @param pose_vec Rays are calculated  using this pose vector [x,y,z,w,x,y,z] and expanded FoV.
 * @return Focus point for the given view (pose)
 */
Eigen::Vector3d focus_point_cls::get_focus_point(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot)
{

  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;


  double r_ray_cast=params_.r_max;
  float r_max_last=params_.focus_point_r;
  
  double best_ray_entropy=-1e100;
  Eigen::Vector3d focus_point(0,0,0);
  Eigen::Vector4d end_point_dq(0,0,0,0);
  octomap::KeyRay ray;
  for (int i = 0; i < static_cast<int>(unit_rays_expanded_view.size()); ++i)
    {
      const auto& u = unit_rays_expanded_view[i];
      Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
      end_point_dq = vec4(translation(pose_dq*(1+E_*0.5*(ray_vec.x()*i_+ray_vec.y()*j_+ray_vec.z()*k_))));

      octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
      octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
      octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
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
      
          focus_point[0] =  r_max_last * u.x();
          focus_point[1] =  r_max_last * u.y();
          focus_point[2]=   r_max_last * u.z();
          focus_point=vec3(translation(pose_dq*(1+E_*0.5*(focus_point[0] *i_+focus_point[1]*j_+focus_point[2]*k_))));
        }

    }
      
    return focus_point;

}

/**
 * Calculates the focus point from the given view (parallel version using OpenMP). Focus point is the point along the ray with maximum entropy.
 * 
 * @param pose_vec Rays are calculated  using this pose vector [x,y,z,w,x,y,z] and expanded FoV.
 * @return Focus point for the given view (pose)
 */

Eigen::Vector3d focus_point_cls::get_focus_point_parallel(std::vector<float>& pose_vec, std::shared_ptr<octomap::OcTree>& ot)
{
  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;

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
        Eigen::Vector4d end_point_dq = vec4(translation(pose_dq*(1+E_*0.5*(ray_vec.x()*i_+ray_vec.y()*j_+ray_vec.z()*k_))));
       
        octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
        octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
        octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
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
          Eigen::Vector3d fp;
          fp[0] =  r_max_last * u.x();
          fp[1] =  r_max_last * u.y();
          fp[2] =  r_max_last * u.z();
          thread_focus_point = vec3(translation(pose_dq*(1+E_*0.5*(fp[0] *i_+fp[1]*j_+fp[2]*k_))));
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

/**
 * Service function to calculate the frontier voxels in the octomap.
 *
 * @param req Service request.
 * @param resp Service response containing the frontier voxel coordinates.
 */
bool focus_point_cls::calculate_frontier(focus_point_calculator::get_frontier_srv::Request &req, focus_point_calculator::get_frontier_srv::Response &resp)
{
  bool use_low_res=params_.use_low_res;
  std::shared_ptr<octomap::OcTree> ot;
  {
      std::lock_guard<std::mutex> lock(octomap_mutex);
        
        if(use_low_res)
        {
          // Lazy rebuild: only rebuild if stale
          if (lowres_needs_rebuild)
          {
            rebuild_lowres_tree();
            lowres_needs_rebuild = false;
          }
          ot = ot_lowres_;
        }
        else
          ot = ot_;
  }
  // Ensure indices are aligned to the current frontier set only
  frontier_voxels.clear();
  std::vector<octomap::OcTreeKey> f;
  extract_frontier(f, ot);
  for(auto& k:f){
    auto p = ot->keyToCoord(k);
    
    if (is_inside_boundaries(Eigen::Vector4d(p.x(),p.y(),p.z(),0))){
      resp.frontier_xyz.insert(resp.frontier_xyz.end(),
      {p.x(),p.y(),p.z()});
      frontier_voxels.push_back(k);
    }
  }
  return true;

}
/**
 * Extracts frontier voxels from the octomap.
 *
 * @param frontier Output vector to store the frontier voxel keys.
 * @param ot Shared pointer to the octomap.
 */
void focus_point_cls::extract_frontier(
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
 * Service function to calculate visible frontier voxels from a given view.
 *
 * @param req Service request containing the view pose.
 * @param resp Service response containing the indices of visible frontier voxels.
 */
bool focus_point_cls::calculate_view_frontier(focus_point_calculator::get_view_frontier_srv::Request &req, focus_point_calculator::get_view_frontier_srv::Response &resp)
{
  
  bool use_low_res=params_.use_low_res;
    std::shared_ptr<octomap::OcTree> ot;
    {
        std::lock_guard<std::mutex> lock(octomap_mutex);
          
          if(use_low_res)
          {
            // Lazy rebuild: only rebuild if stale
            if (lowres_needs_rebuild)
            {
              rebuild_lowres_tree();
              lowres_needs_rebuild = false;
            }
            ot = ot_lowres_;
          }
          else
            ot = ot_;
    }

  if (frontier_voxels.empty()) 
  {
    resp.frontier_indices.clear();
    return true;
  }

  std::unordered_map<uint64_t,int> key2idx;
  for (size_t i=0;i<frontier_voxels.size();++i)
  {
    const auto& k = frontier_voxels[i];
    // Create unique ID from key components
    uint64_t hash = (uint64_t(k.k[0]) << 42) | (uint64_t(k.k[1]) << 21) | uint64_t(k.k[2]);
    key2idx[hash] = i;
  }
  std::vector<float> req_vec=req.pose;
  std::vector<int> visible_indices;
  bool run_parallel=req.run_parallel;

  if(run_parallel)
  {
  compute_visible_frontier_parallel(req_vec, key2idx, visible_indices, ot);
  }
  else
  {
  compute_visible_frontier(req_vec, key2idx, visible_indices, ot);
  }
  
  // Populate response with visible frontier indices
  resp.frontier_indices = visible_indices;
  
  ROS_INFO_STREAM("Found " << visible_indices.size() << " visible frontier voxels out of " << frontier_voxels.size() << " total frontier voxels");
  
  return true;
}

/**
 * Computes visible frontier voxels from a given pose.
 *
 * @param pose_vec Pose vector [x,y,z,w,x,y,z].
 * @param key2idx Unordered map mapping voxel key hashes to frontier indices.
 * @param out_indices Output vector of visible frontier indices.
 * @param ot Octomap to query for ray casting.
 */
void focus_point_cls::compute_visible_frontier(
  std::vector<float>& pose_vec,
  std::unordered_map<uint64_t,int>& key2idx,
  std::vector<int>& out_indices, std::shared_ptr<octomap::OcTree>& ot)
{
  out_indices.clear();

  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;

  double r_ray_cast=params_.r_max;
  Eigen::Vector3d vec;
  Eigen::Vector4d end_point_dq;

  int total_rays = 0;
  int total_keys_checked = 0;
  octomap::KeyRay ray;
  for (int i = 0; i < static_cast<int>(unit_rays_normal_view.size()); ++i)
    {
      const auto& u = unit_rays_normal_view[i];
      Eigen::Vector3d ray_vec(u.x()*r_ray_cast, u.y()*r_ray_cast, u.z()*r_ray_cast);
      end_point_dq = vec4(translation(pose_dq*(1+E_*0.5*(ray_vec.x()*i_+ray_vec.y()*j_+ray_vec.z()*k_))));
      octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
      octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
      octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
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
 * @param pose_vec Pose vector [x,y,z,w,x,y,z].
 * @param key2idx Unordered map mapping voxel key hashes to frontier indices.
 * @param out_indices Output vector of visible frontier indices.
 * @param ot Octomap to query for ray casting.
 */
void focus_point_cls::compute_visible_frontier_parallel(
  std::vector<float>& pose_vec,
  std::unordered_map<uint64_t,int>& key2idx,
  std::vector<int>& out_indices, std::shared_ptr<octomap::OcTree>& ot)
{
  out_indices.clear();
  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;

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
        Eigen::Vector4d end_point_dq = vec4(translation(pose_dq*(1+E_*0.5*(ray_vec.x()*i_+ray_vec.y()*j_+ray_vec.z()*k_))));

        octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
        octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
        octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
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
void focus_point_cls::calculate_unit_ray_set(std::vector<Eigen::Vector3d>& unit_ray_set, double hfov, double vfov, double dphi, double dtheta)
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
 * Calculates octomap related metrics such as entropy, covered volume = free+occupied volume, the number of occupied, free, and unknown 
 * voxels within specified boundaries.
 *
 * @param req Service request.
 * @param resp Service response.
 */

bool focus_point_cls::calculate_occluded_volume(focus_point_calculator::coverage_srv::Request &req, focus_point_calculator::coverage_srv::Response &resp)
{
  
  std::shared_ptr<octomap::OcTree> ot;
    {
        std::lock_guard<std::mutex> lock(octomap_mutex);
        ot = ot_;
    }
  float x_max(boundary_max[0]);
  float x_min(boundary_min[0]);
  float y_max(boundary_max[1]);
  float y_min(boundary_min[1]);
  float z_max(boundary_max[2]);
  float z_min(boundary_min[2]);
  float occupied(0);
  float free(0);
  float unknown(0);
  float ent(0);
  float cov(0);
  double res=params_.cell_size;
  double dV = res*res*res;
  double prob_occ;
  double comp_prob;
  double curr_ent(0);
  for (float x=x_min;x<x_max;x=x+res)
    for (float y=y_min;y<y_max;y=y+res)
      for (float z=z_min;z<z_max;z=z+res)
      {
          Eigen::Vector4d v(x, y, z, 0);
          if (!is_inside_boundaries(v))
               continue;
          octomap::OcTreeNode* result = ot->search(x+res/2,y+res/2,z+res/2);
          
          if (result)
          { 
          cov+=dV; //Coverage free+occupied volumes
          prob_occ=result->getOccupancy(); //Probability of the voxel being occupied
          comp_prob=1-prob_occ; //Complement probability ( the voxel being free)
          curr_ent=entropy(prob_occ);

          if(result->getLogOdds() > 0)
             occupied+=1;
          else
             free+=1;
       
          }
          else
          {
          unknown+=1;
          // If the voxel is unknown, probability is 0.5
          curr_ent=entropy(0.5);
         }

          ent+=curr_ent;
      }
 resp.cvr=cov;
 resp.ent=ent;
 resp.unknown=unknown;
 resp.occupied=occupied;
 resp.free=free;

 return true;

}


/**
 * Save the current octomap as .bt file
 *
 */
void focus_point_cls::save_tree()
{
  std::shared_ptr<octomap::OcTree> ot;
    {
        std::lock_guard<std::mutex> lock(octomap_mutex);
        ot = ot_;
    }
  std::string filname=params_.octomap_save_path+std::to_string(counter)+".bt";
  ot->writeBinary(filname);
}

/**
 * Save the current octomap to a specified file path.
 *
 * @param req Service request containing file_path.
 * @param resp Service response with success status and message.
 */
bool focus_point_cls::save_octomap_to_path(focus_point_calculator::save_octomap_srv::Request &req,
                                           focus_point_calculator::save_octomap_srv::Response &resp)
{
  if (!is_octomap_received)
  {
    resp.success = false;
    resp.message = "No octomap received yet";
    ROS_WARN("save_octomap service called but no octomap has been received");
    return true;
  }

  std::shared_ptr<octomap::OcTree> ot;
  {
    std::lock_guard<std::mutex> lock(octomap_mutex);
    ot = ot_;
  }

  if (!ot)
  {
    resp.success = false;
    resp.message = "Octomap pointer is null";
    ROS_ERROR("Octomap pointer is null in save_octomap_to_path");
    return true;
  }

  try
  {
    std::string file_path = req.file_path;
    
    // Ensure the file has .bt extension
    if (file_path.size() < 3 || file_path.substr(file_path.size() - 3) != ".bt")
    {
      file_path += ".bt";
    }

    bool write_success = ot->writeBinary(file_path);
    
    if (write_success)
    {
      resp.success = true;
      resp.message = "Octomap saved successfully to " + file_path;
      ROS_INFO_STREAM("Octomap saved to: " << file_path);
    }
    else
    {
      resp.success = false;
      resp.message = "Failed to write octomap to " + file_path;
      ROS_ERROR_STREAM("Failed to write octomap to: " << file_path);
    }
  }
  catch (const std::exception& e)
  {
    resp.success = false;
    resp.message = std::string("Exception while saving octomap: ") + e.what();
    ROS_ERROR_STREAM("Exception in save_octomap_to_path: " << e.what());
  }

  return true;
}

/**
 * Check if the given point inside the boundaries
 *
 * @param point 3D point to check if it is inside boundaries.
 * @return True is the point is inside the boundaries.
 */
bool focus_point_cls::is_inside_boundaries(Eigen::Vector4d point)
{
  return point[0] > boundary_min[0] and point[0] < boundary_max[0] and
         point[1] > boundary_min[1] and point[1] < boundary_max[1] and
         point[2] > boundary_min[2] and point[2] < boundary_max[2];
}
