#include <niryo_one_tutorial/track_pose_service.h>

TrackPoseService::TrackPoseService(DomusInterface* domus_interface, ros::NodeHandle* n) : controller(domus_interface, n)
{
  is_active = false;
}

void TrackPoseService::run_tracking()
{
  ros::Duration sleep_time(0.5);
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
    sleep_time.sleep();
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
  bool isSimulation;
  ros::param::get("~sim", isSimulation);

  DomusInterface* domus_interface;
  if (isSimulation){
    ROS_INFO_STREAM("Whether we are simulating" << isSimulation << std::endl);
    domus_interface = new MockDomusInterface();
  } else {
    domus_interface = new DomusInterface();
  }
  ros::AsyncSpinner spinner(1); // use 1 thread async for callbacks
  spinner.start();
  std::cout << "Waiting for DomusInterface in case it's slow to come up";
  ros::Duration(5).sleep();
  TrackPoseService trackPoseService(domus_interface, &n);
  std::cout << "Waiting for trackPoseService in case it's slow to come up";
  ros::Duration(5).sleep();
  ros::ServiceServer service = n.advertiseService("update_pose_target", &TrackPoseService::handle_target_update, &trackPoseService);
  trackPoseService.run_tracking();

  delete domus_interface;
  return 0;
}
