#include "octomap_operations/octomap_ops.hpp"
using namespace std;
using namespace octomap;

octomap_ops_cls::octomap_ops_cls(const ros::NodeHandle &rosNode):rosNode_(rosNode)
{
  params_ = readParams();
  boundary_max=params_.boundary_max;
  boundary_min=params_.boundary_min;
  octo_map_sub = rosNode_.subscribe(params_.octomap_topic_name, 1, &octomap_ops_cls::octomap_callback, this);
  // Subscribe to coarse octomap if topic is configured (not empty)
  if (params_.raycast_use_low_res_octomap && !params_.lowres_octomap_topic_name.empty()) {
    octo_map_coarse_sub = rosNode_.subscribe(params_.lowres_octomap_topic_name, 1, &octomap_ops_cls::octomap_coarse_callback, this);
    ROS_INFO_STREAM("Subscribed to coarse octomap: " << params_.lowres_octomap_topic_name);
  }
  coverage_service=rosNode_.advertiseService("get_coverage",&octomap_ops_cls::calculate_occluded_volume,this);
  focus_point_service=rosNode_.advertiseService("get_focus_point",&octomap_ops_cls::calculate_focus_point,this);
  view_evaluate_service=rosNode_.advertiseService("get_view_igs",&octomap_ops_cls::calculate_view_igs,this);
  save_octomap_service=rosNode_.advertiseService("save_octomap",&octomap_ops_cls::save_octomap_to_path,this);
  get_frontier_service=rosNode_.advertiseService("get_frontier",&octomap_ops_cls::calculate_frontier,this);
  get_view_frontier_service=rosNode_.advertiseService("get_view_frontier",&octomap_ops_cls::calculate_view_frontier,this);
  stop_octomap_update_service=rosNode_.advertiseService("stop_octomap_update",&octomap_ops_cls::stop_octomap_update,this);
  is_octomap_received=false;
  counter=0;
  calculate_unit_ray_set(unit_rays_normal_view,params_.hfov,params_.vfov,params_.dphi,params_.dtheta);
  calculate_unit_ray_set(unit_rays_expanded_view,params_.hfov_expanded,params_.vfov_expanded,params_.dphi,params_.dtheta);
  
}

/**
 * This function is called when octomap data is published.
 *
 * @param msg Octomap message.
 */
void octomap_ops_cls::octomap_callback(const octomap_msgs::Octomap& msg)
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
  ROS_DEBUG_STREAM("Octomap updated");
}

// Coarse octomap callback
void octomap_ops_cls::octomap_coarse_callback(const octomap_msgs::Octomap& msg)
{
  if (block_octomap_updates)
  {
    ROS_DEBUG_STREAM("Coarse octomap update blocked by user");
    return;
  }

  ROS_DEBUG_STREAM("Received coarse octomap message");
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = dynamic_cast<octomap::OcTree*>(aot);
  if (!ot)
  {
      ROS_ERROR("Received coarse Octomap is not an OcTree");
      delete aot;
      return;
  }
  {
    std::lock_guard<std::mutex> lock(coarse_octomap_mutex);
    ot_coarse_ = std::make_shared<octomap::OcTree>(*ot);
  }
  delete ot;
  is_coarse_octomap_received = true;
  ROS_DEBUG_STREAM("Coarse Octomap updated");
}

/**
 * Service function to stop or resume octomap updates.
 *
 * @param req Service request.
 * @param resp Service response.
 */ 
bool octomap_ops_cls::stop_octomap_update(orac_reconstruction_services::stop_octomap_update_srv::Request &req, orac_reconstruction_services::stop_octomap_update_srv::Response &resp)
{
  block_octomap_updates=req.block_updates;
  if(block_octomap_updates)
    ROS_INFO("Octomap updates have been blocked.");
  else
    ROS_INFO("Octomap updates have been unblocked.");
  resp.success=true;
  return true;
}
/* Low-resolution rebuild removed. Using separate coarse octomap subscription when enabled. */

/**
 * Service function to calculate the information gain (IG) for a list of views.
 * @param req Service request containing the list of views and IG method.
 * @param resp Service response containing the calculated IGs for each view.
 */
 bool octomap_ops_cls::calculate_view_igs(orac_reconstruction_services::view_evaluate_srv::Request &req,orac_reconstruction_services::view_evaluate_srv::Response &resp)
 {
  bool use_low_res=params_.raycast_use_low_res_octomap;
  if(is_octomap_received)
  {
  std::shared_ptr<octomap::OcTree> ot;
  {
        
        if (use_low_res && is_coarse_octomap_received)
        {
          std::lock_guard<std::mutex> lock(coarse_octomap_mutex);
          ot = ot_coarse_;
        }
        else
        { 
          std::lock_guard<std::mutex> lock(octomap_mutex);
          ot = ot_;
        }
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
 * It is service function which calculates the focus point.
 *
 * @param req Service request.
 * @param resp Service response.
 */

bool octomap_ops_cls::calculate_focus_point(orac_reconstruction_services::focus_point_srv::Request &req,orac_reconstruction_services::focus_point_srv::Response &resp)
{
  bool use_low_res=params_.raycast_use_low_res_octomap;
  if(is_octomap_received)
  {
  std::shared_ptr<octomap::OcTree> ot;
  {
        
        if (use_low_res && is_coarse_octomap_received)
        {
          std::lock_guard<std::mutex> lock(coarse_octomap_mutex);
          ot = ot_coarse_;
        }
        else
        {
          std::lock_guard<std::mutex> lock(octomap_mutex);
          ot = ot_;
        }
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
 * Service function to calculate the frontier voxels in the octomap.
 *
 * @param req Service request.
 * @param resp Service response containing the frontier voxel coordinates.
 */
bool octomap_ops_cls::calculate_frontier(orac_reconstruction_services::get_frontier_srv::Request &req, orac_reconstruction_services::get_frontier_srv::Response &resp)
{
  bool use_low_res=params_.raycast_use_low_res_octomap;
  std::shared_ptr<octomap::OcTree> ot;
  {
      if (use_low_res && is_coarse_octomap_received)
      {
        std::lock_guard<std::mutex> lock(coarse_octomap_mutex);
        ot = ot_coarse_;
      }
      else
      {
        std::lock_guard<std::mutex> lock(octomap_mutex);
        ot = ot_;
      }
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
 * Service function to calculate visible frontier voxels from a given view.
 *
 * @param req Service request containing the view pose.
 * @param resp Service response containing the indices of visible frontier voxels.
 */
bool octomap_ops_cls::calculate_view_frontier(orac_reconstruction_services::get_view_frontier_srv::Request &req, orac_reconstruction_services::get_view_frontier_srv::Response &resp)
{
  
  bool use_low_res=params_.raycast_use_low_res_octomap;
    std::shared_ptr<octomap::OcTree> ot;
    {
        
          if (use_low_res && is_coarse_octomap_received)
          {
            std::lock_guard<std::mutex> lock(coarse_octomap_mutex);
            ot = ot_coarse_;
          }
          else
          {
            std::lock_guard<std::mutex> lock(octomap_mutex);
            ot = ot_;
          }
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
 * Calculates octomap related metrics such as entropy, covered volume = free+occupied volume, the number of occupied, free, and unknown 
 * voxels within specified boundaries.
 *
 * @param req Service request.
 * @param resp Service response.
 */

bool octomap_ops_cls::calculate_occluded_volume(orac_reconstruction_services::coverage_srv::Request &req, orac_reconstruction_services::coverage_srv::Response &resp)
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
void octomap_ops_cls::save_tree()
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
bool octomap_ops_cls::save_octomap_to_path(orac_reconstruction_services::save_octomap_srv::Request &req,
                                           orac_reconstruction_services::save_octomap_srv::Response &resp)
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