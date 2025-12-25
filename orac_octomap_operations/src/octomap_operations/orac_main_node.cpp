#include <octomap_operations/octomap_ops.hpp>
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_ops_node");
  ros::NodeHandle nh;

  octomap_ops_cls octomap_ops_node(nh);
  ros::spin();

  return 0;
}