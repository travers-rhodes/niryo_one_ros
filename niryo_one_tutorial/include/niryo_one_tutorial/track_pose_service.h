#include <ros/ros.h>
#include <niryo_one_tutorial/jacobian_controller.h>
#include "niryo_one_tutorial/TrackPose.h"
#include "niryo_one_tutorial/domus_interface.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "geometry_msgs/Pose.h"

class TrackPoseService
{
  public:
    TrackPoseService();
    bool handle_target_update(niryo_one_tutorial::TrackPose::Request &req,
           niryo_one_tutorial::TrackPose::Response &res);
    void run_tracking();
 
  private:
    geometry_msgs::Pose target_pose;
    JacobianController controller;
    bool is_active;
};

