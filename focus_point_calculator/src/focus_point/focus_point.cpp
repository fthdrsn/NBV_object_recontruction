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
  coverage_service=rosNode_.advertiseService("get_coverage",&focus_point_cls::calculate_ocluded_volume,this);
  focus_point_service=rosNode_.advertiseService("get_focus_point",&focus_point_cls::calculate_focus_point,this);
  view_evaluate_service=rosNode_.advertiseService("get_view_igs",&focus_point_cls::calculate_view_igs,this);
  is_camera_initiated=false;
  is_octomap_received=false;
  counter=0;
}

/**
 * This function is called when octomap data is published.
 *
 * @param msg Octomap message.
 */
void focus_point_cls::octomap_callback(const octomap_msgs::Octomap& msg)
{
  is_octomap_received=true;
  ROS_DEBUG_STREAM("Freeing ot_");
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  delete ot;
  ROS_DEBUG_STREAM("Freeing ot_ done:");
}

/**
 * It is service function which calculates the focus point.
 *
 * @param req Service request.
 * @param resp Service response.
 */

bool focus_point_cls::calculate_focus_point(focus_point_calculator::focus_point_srv::Request &req,focus_point_calculator::focus_point_srv::Response &resp)
{
  
  if(is_octomap_received)
  {
  std::cout<<"Start focus point calculation"<<std::endl;
  std::vector<float> req_vec=req.pose;

  std::vector<float> tmp_vec;
  std::vector<float> result_vec;
  Eigen::Vector3d focus_pnt;

  focus_pnt=get_focus_point(req_vec);
 
  result_vec.insert(result_vec.end(),focus_pnt[0]);
  result_vec.insert(result_vec.end(),focus_pnt[1]);
  result_vec.insert(result_vec.end(),focus_pnt[2]);
  resp.focus_pnt=result_vec;
  counter++;
 
 }else
 {
 resp.focus_pnt={};
 }
  
  return true;
}


 bool focus_point_cls::calculate_view_igs(focus_point_calculator::view_evaluate_srv::Request &req,focus_point_calculator::view_evaluate_srv::Response &resp)
 {
  
  if(is_octomap_received)
  {
  std::vector<float> reqVec=req.view_list;
  string ig_method=req.ig_method;
  std::vector<float> tmpVec;
  std::vector<float> resultVec;
  double ig;
  double totalTime=0;
  for(int i = 0; i < reqVec.size(); i++){ 
  tmpVec.insert(tmpVec.end(),reqVec[i]);
  if(tmpVec.size()==7)
  {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if (ig_method == "RSV")
    {

    ig=get_view_ig_rsv(tmpVec);
    }
    else if(ig_method == "ENT")
    {
    ig=get_view_ig_ent(tmpVec);
    }
    else{
     cout<<"Not valid IG method is provided";
     resp.view_igs={};
     return false;
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    totalTime+=std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    resultVec.insert(resultVec.end(),ig);
    tmpVec.clear();
  }
 }
  cout<<"Average Time: "<<totalTime<<endl;
  resp.view_igs=resultVec;
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
 * Calculates the ig of the given view.
 *
 * @param pose_vec Focus vector is calculated for this pose vector [x,y,z,w,x,y,z].
 * @return IG for the given view (pose)
 */

double focus_point_cls::get_view_ig_ent(std::vector<float> pose_vec)
{

  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;

  
  std::shared_ptr<octomap::OcTree> ot = ot_;
  double gain = 0.0;
  double fov_y = params_.hfov, fov_p = params_.vfov;
  double dphi = params_.dphi, dtheta = params_.dtheta;

  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  Eigen::Vector3d vec, dir;
  Eigen::Vector4d end_point_dq;


  float r_max_last=params_.focus_point_r;
  float best_g=0;
  Eigen::Vector3d focus_point;
  double r_ray_cast=params_.r_max;
  double g=0;
  double view_entropy=0;


     /*
        y  
        |
  x_____|    
  */
  for (theta = -fov_y/2; theta < fov_y/2; theta += dtheta) //right to left
  {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = -fov_p / 2; phi < fov_p / 2; phi += dphi) // up to down
    {

        // Calculate information gain for a ray
        phi_rad = M_PI * phi / 180.0f;

        vec[0] = r_ray_cast * sin(theta_rad) * cos(phi_rad);
        vec[1] = -r_ray_cast * cos(theta_rad) * sin(phi_rad);
        vec[2] = r_ray_cast * cos(theta_rad) * cos(phi_rad);
        end_point_dq=vec4(translation(pose_dq*(1+E_*0.5*(vec[0] *i_+vec[1]*j_+vec[2]*k_))));

        octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
        octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
        octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
      
        double max_range=params_.r_max;
        bool found_endpoint = ot->castRay( origin, direction, end_point, true, max_range ); // ignore unknown cells
        //true if an occupied cell was hit, false if the maximum range or octree bounds are reached

        octomap::KeyRay ray;
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


          view_entropy+=(-prob_occupancy*log(prob_occupancy)-(1-prob_occupancy)*(log(1-prob_occupancy)));
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

          view_entropy+=(-prob_occupancy*log(prob_occupancy)-(1-prob_occupancy)*(log(1-prob_occupancy)));
          
        }     


    }
      
  }
    return view_entropy;

}

/**
 * Calculates the ig of the given view.
 *
 * @param pose_vec Focus vector is calculated for this pose vector [x,y,z,w,x,y,z].
 * @return IG for the given view (pose)
 */

double focus_point_cls::get_view_ig_rsv(std::vector<float> pose_vec)
{

  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;

  
  std::shared_ptr<octomap::OcTree> ot = ot_;
  double gain = 0.0;
  double fov_y = params_.hfov, fov_p = params_.vfov;
  double dphi = params_.dphi, dtheta = params_.dtheta;
  //double dphi = 5, dtheta = 5;

  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  Eigen::Vector3d vec, dir;
  Eigen::Vector4d end_point_dq;


  float r_max_last=params_.focus_point_r;
  float best_g=0;
  Eigen::Vector3d focus_point;
  double r_ray_cast=params_.r_max;
  double g=0;
  double rear_side_voxel_count=0;

  /*
        y  
        |
  x_____|    
  */
  for (theta = -fov_y/2; theta < fov_y/2; theta += dtheta) //  right to left
  {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = -fov_p / 2; phi < fov_p / 2; phi += dphi) // up to down
    {
        // Calculate information gain for a ray
        phi_rad = M_PI * phi / 180.0f;

        vec[0] = r_ray_cast * sin(theta_rad) * cos(phi_rad);
        vec[1] = -r_ray_cast * cos(theta_rad) * sin(phi_rad);
        vec[2] = r_ray_cast * cos(theta_rad) * cos(phi_rad);
        end_point_dq=vec4(translation(pose_dq*(1+E_*0.5*(vec[0] *i_+vec[1]*j_+vec[2]*k_))));

        octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
        octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
        octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
        double max_range=params_.r_max;
        bool found_endpoint = ot->castRay(origin, direction, end_point, true, max_range); // ignore unknown cells
        //true if an occupied cell was hit, false if the maximum range or octree bounds are reached
        octomap::KeyRay ray;
        octomap::OcTreeNode* prev_node;
        if(found_endpoint) //The ray should hit an occupied voxel, otherwise there is no rear_side_voxel on this ray
        {
          ot->computeRayKeys(origin, end_point, ray); //Compute octree keys for all traversed voxels (excluding the end_point)
          prev_node =ot->search(*ray.end()); //search the previous voxel of the end point
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
 * Calculates the focus point for the given pose.
 *
 * @param pose_vec Focus vector is calculated for this pose vector [x,y,z,w,x,y,z].
 * @return 3D focus point
 */

Eigen::Vector3d focus_point_cls::get_focus_point(std::vector<float> pose_vec)
{

  DQ tr=pose_vec[0]*i_+pose_vec[1]*j_+pose_vec[2]*k_;
  DQ ori=normalize(pose_vec[3] +pose_vec[4]*i_+pose_vec[5]*j_+pose_vec[6]*k_);
  DQ pose_dq=ori+E_*0.5*tr*ori;
 
  std::shared_ptr<octomap::OcTree> ot = ot_;
  double gain = 0.0;
  double fov_y = params_.hfov_expanded, fov_p = params_.vfov_expanded;
  double dphi = params_.dphi, dtheta = params_.dtheta;

  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  Eigen::Vector3d vec, dir;
  Eigen::Vector4d end_point_dq;


  float r_max_last=params_.focus_point_r;
  float best_ray_entropy=0;
  Eigen::Vector3d focus_point;
  double r_ray_cast=params_.r_max;

  for (theta = -fov_y/2; theta < fov_y/2; theta += dtheta) //right to left
  {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = - fov_p / 2; phi < (fov_p / 2-15); phi += dphi) //Up to Down
    {
        // Calculate information gain for a ray
        phi_rad = M_PI * phi / 180.0f;
        
        vec[0] = r_ray_cast * sin(theta_rad) * cos(phi_rad);
        vec[1] = -r_ray_cast * cos(theta_rad) * sin(phi_rad);
        vec[2] = r_ray_cast * cos(theta_rad) * cos(phi_rad);
        end_point_dq=vec4(translation(pose_dq*(1+E_*0.5*(vec[0] *i_+vec[1]*j_+vec[2]*k_))));

        octomap::point3d origin(pose_vec[0],pose_vec[1],pose_vec[2]);
        octomap::point3d end_point(end_point_dq[1], end_point_dq[2],end_point_dq[3]);
        octomap::point3d direction(end_point_dq[1]-pose_vec[0], end_point_dq[2]-pose_vec[1],end_point_dq[3]-pose_vec[2]);
        double max_range=params_.r_max;
        bool found_endpoint = ot->castRay( origin, direction, end_point, true, max_range ); // ignore unknown cells
        octomap::KeyRay ray;
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


          ray_entropy+=(-prob_occupancy*log(prob_occupancy)-(1-prob_occupancy)*(log(1-prob_occupancy)));
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

          ray_entropy+=(-prob_occupancy*log(prob_occupancy)-(1-prob_occupancy)*(log(1-prob_occupancy)));
          
        }     


        if(ray_entropy>best_ray_entropy)
        {
          best_ray_entropy=ray_entropy;
          focus_point[0] =  r_max_last * sin(theta_rad) * cos(phi_rad);
          focus_point[1] =  -r_max_last * cos(theta_rad) * sin(phi_rad);
          focus_point[2]=   r_max_last * cos(theta_rad) * cos(phi_rad);
          focus_point=vec3(translation(pose_dq*(1+E_*0.5*(focus_point[0] *i_+focus_point[1]*j_+focus_point[2]*k_))));
        }

    }
      
  }
    return focus_point;

}


/**
 * Calculates the occluded volume in specified boundaries. It also calculates the entropy inside the boundaries.
 *
 * @param req Service request.
 * @param resp Service response.
 */

bool focus_point_cls::calculate_ocluded_volume(focus_point_calculator::coverage_srv::Request &req, focus_point_calculator::coverage_srv::Response &resp)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;
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
          octomap::OcTreeNode* result = ot_->search(x+res/2,y+res/2,z+res/2);
          
          if (result)
          { 
          cov+=dV; //Coverage free+occupied volumes
          prob_occ=result->getOccupancy(); //Probability of the voxel being occupied
          comp_prob=1-prob_occ; //Complement probability ( the voxel being free)
          curr_ent=-prob_occ*log(prob_occ)-comp_prob*log(comp_prob);

          if(result->getLogOdds() > 0)
             occupied+=1;
          else
             free+=1;
       
          }
          else
          {
          unknown+=1;
          // If the voxel is unknown, probability is 0.5
          curr_ent=-0.5*log(0.5)-(0.5)*log(0.5);
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
  string filname=params_.octomap_save_path+std::to_string(counter)+".bt";
  ot_->writeBinary(filname);
}

/**
 * Check is the given point inside the boundaries
 *
 * @param point 3D point to check if it is inside boundaries.
 * @return True is the point is inside the boundaries.
 */
bool focus_point_cls::is_inside_boundaries(Eigen::Vector4d point)
{
  //double dist= sqrt(point[0]*point[0]+point[1]*point[1]);
  //return dist<2.75 and point[2] > boundary_min[2] and point[2] < boundary_max[2];
  return point[0] > boundary_min[0] and point[0] < boundary_max[0] and
         point[1] > boundary_min[1] and point[1] < boundary_max[1] and
         point[2] > boundary_min[2] and point[2] < boundary_max[2];
}
