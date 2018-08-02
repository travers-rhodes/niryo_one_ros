//
// This ros node starts a server at "track_pose_server" which exports a service
// "update_pose_target". The API of that service call is, if "stopMotion" is set
// on the request, stop the robot
// and otherwise, start the robot moving toward the given target pose
// this service also periodically publishes to "/distance_to_target"
//
#include <ros/ros.h>
#include <niryo_one_tutorial/jacobian_controller.h>
#include "niryo_one_tutorial/TrackPose.h"
#include "niryo_one_tutorial/domus_interface.h"
#include "niryo_one_tutorial/mock_domus_interface.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "geometry_msgs/Pose.h"

class TrackPoseService
{
  public:
    TrackPoseService(double update_rate_hz, double step_size_meters, DomusInterface* domus_interface, ros::NodeHandle*);
    bool handle_target_update(niryo_one_tutorial::TrackPose::Request &req,
           niryo_one_tutorial::TrackPose::Response &res);
    void run_tracking();
 
  private:
    geometry_msgs::Pose target_pose;
    JacobianController controller;
    bool is_active;
    double _update_rate_hz;
    ros::Publisher dist_pub_;
};

