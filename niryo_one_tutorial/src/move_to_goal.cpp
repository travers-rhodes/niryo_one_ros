#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Dense>
#include <ros/ros.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_to_goal");
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  std::cout << "Model frame: " <<  kinematic_model->getModelFrame().c_str() << std::endl;

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  //const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

  Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform("ground_link");
  
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;

  kinematic_state->getJacobian(joint_model_group, 
    kinematic_state->getLinkModel("hand_link"),
    reference_point_position,
    jacobian);

  ROS_ERROR_STREAM("Jacobian" << jacobian << std::endl);
}
