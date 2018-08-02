//
// This ROS node listens on /set_joint_angles and will move DOMUS to the requested position in joint angles
//

#include <ros/ros.h>
#include <niryo_one_tutorial/domus_interface.h>
#include <niryo_one_tutorial/JointAngles.h>

DomusInterface domus;

void set_angles_callback(const niryo_one_tutorial::JointAngles::ConstPtr& msg)
{
  ROS_INFO_STREAM("I heard" << msg->joint_angles[0] << msg->joint_angles[1] <<"and more");
  //https://stackoverflow.com/questions/6399090/c-convert-vectorint-to-vectordouble
  std::vector<double> joint_angle_vector(msg->joint_angles.begin(), msg->joint_angles.end());
  domus.SendTargetAngles(joint_angle_vector);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "domus_controller");
  ros::NodeHandle n;
  domus.InitializeConnection();
  ros::Subscriber sub = n.subscribe("set_joint_angles", 10, set_angles_callback);
  ros::spin();
  return 0; 
}
