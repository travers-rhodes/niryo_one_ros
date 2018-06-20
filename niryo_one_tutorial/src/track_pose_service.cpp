#include <ros/ros.h>
#include <niryo_one_tutorial/jacobian_controller.h>
#include "niryo_one_tutorial/TrackPose.h"
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
    geometry_msgs::Pose *target_pose;
    JacobianController *controller;    
};

TrackPoseService::TrackPoseService()
{
  JacobianController new_controller;
  controller = &new_controller;
}

void TrackPoseService::run_tracking()
{
  ros::Rate r(50);
  while (ros::ok())
  {
    try
    {
      if(target_pose)
      {
        controller->make_step_to_target_pose(*target_pose);
      }
    }
    catch(...)
    {
      std::cout << "You hit an error!";
      throw;
    }
    r.sleep();
  }
}

bool TrackPoseService::handle_target_update(niryo_one_tutorial::TrackPose::Request &req,
                          niryo_one_tutorial::TrackPose::Response &res)
{
  if(req.stopMotion)
  {
    target_pose = NULL;
  }
  else
  {
    geometry_msgs::Pose target_copy(req.target);
    target_pose = &target_copy;
  }
  res.success = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_pose_server");
  ros::NodeHandle n;

  TrackPoseService trackPoseService;
  ros::ServiceServer service = n.advertiseService("update_pose_target", &TrackPoseService::handle_target_update, &trackPoseService);
  ros::spin();

  return 0;
}
