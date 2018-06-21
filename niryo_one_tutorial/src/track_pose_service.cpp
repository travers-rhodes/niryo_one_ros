#include <niryo_one_tutorial/track_pose_service.h>

TrackPoseService::TrackPoseService() : controller()
{
  is_active = false;
}

void TrackPoseService::run_tracking()
{
  ros::Rate r(1);
  while (ros::ok())
  {
    //std::cout << "running tracking!" << std::endl;
    try
    {
      if(is_active)
      {
        controller.make_step_to_target_pose(target_pose);
      }
    }
    catch(...)
    {
      std::cout << "You hit an error!";
      throw;
    }
    ros::spinOnce();
    r.sleep();
  }
}

bool TrackPoseService::handle_target_update(niryo_one_tutorial::TrackPose::Request &req,
                          niryo_one_tutorial::TrackPose::Response &res)
{
  if(req.stopMotion)
  {
    is_active = false;
  }
  else
  {
    geometry_msgs::Pose target_copy(req.target);
    target_pose = target_copy;
    is_active = true;
  }
  res.success = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_pose_server");
  ros::NodeHandle n;

  TrackPoseService trackPoseService;
  ros::ServiceServer service = n.advertiseService("update_pose_target", &TrackPoseService::handle_target_update, &trackPoseService);
  trackPoseService.run_tracking();

  return 0;
}
