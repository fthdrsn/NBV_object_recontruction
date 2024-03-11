#include <focus_point/focus_point.hpp>
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "focus_point_node");
  ros::NodeHandle nh;

  focus_point_cls calculate_focus_point(nh);
  ros::spin();

  return 0;
}