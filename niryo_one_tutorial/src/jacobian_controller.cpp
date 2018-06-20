#include <niryo_one_tutorial/jacobian_controller.h>

const double TRANS_EPSILON = 0.01;
const double QUAT_EPSILON = 0.01;

double quat_distance(const Eigen::Quaterniond &cur_quat, const Eigen::Quaterniond &end_quat)
{
  Eigen::Quaterniond diff =  cur_quat.inverse() * end_quat;
  return std::sqrt(1.0 - std::pow(diff.w(),2));
}

double distance(const Eigen::Translation3d &cur_trans, const Eigen::Translation3d &end_trans)
{
  Eigen::Translation3d diff = cur_trans.inverse() * end_trans;
  return std::sqrt(std::pow(diff.x(), 2) + std::pow(diff.y(), 2) + std::pow(diff.z(),2));
}

JacobianController::JacobianController()
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  std::cout << "Model frame: " <<  kinematic_model->getModelFrame().c_str() << std::endl;

  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup("arm");
}

void
JacobianController::make_step_to_target_pose(const geometry_msgs::Pose &target_pose)
{
  Eigen::Quaterniond target_quat(target_pose.orientation.w,
                              target_pose.orientation.x,
                              target_pose.orientation.y, 
                              target_pose.orientation.z);
  Eigen::Translation3d target_trans(target_pose.position.x,
                                    target_pose.position.y,
                                    target_pose.position.z);

  Eigen::Translation3d cur_trans(current_pose->translation());
  Eigen::Quaterniond cur_quat(current_pose->rotation());
  double quat_dist = quat_distance(cur_quat, target_quat);
  double trans_dist = distance(cur_trans, target_trans);
  if (trans_dist < TRANS_EPSILON &&
      quat_dist < QUAT_EPSILON)
  {
    //no need to move
    //publish current pose
    return;
  }
  Eigen::Affine3d pseudo_end_pose = get_pseudo_end_pose(target_trans, target_quat);
  move_to_target_pose(pseudo_end_pose);  
}

Eigen::Affine3d
JacobianController::get_pseudo_end_pose(Eigen::Translation3d target_trans, Eigen::Quaterniond target_quat)
{
  Eigen::Affine3d pseudo_end_pose = target_trans * target_quat;
  return pseudo_end_pose;
}

void
JacobianController::move_to_target_pose(const Eigen::Affine3d &target_pose)
{
  return;
}
