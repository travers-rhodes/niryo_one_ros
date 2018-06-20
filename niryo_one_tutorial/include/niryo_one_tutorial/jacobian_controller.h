#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

class JacobianController
{
  public:
    void make_step_to_target_pose(const geometry_msgs::Pose &target_pose);
};
