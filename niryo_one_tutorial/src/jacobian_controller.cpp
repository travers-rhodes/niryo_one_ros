#include <niryo_one_tutorial/jacobian_controller.h>

const double TRANS_EPSILON = 0.01;
const double QUAT_EPSILON = 0.01;
const double ANGLE_STEP_SIZE = 0.01;
const double TRANS_STEP_SIZE = 0.01;
 
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

JacobianController::JacobianController() : robot_model_loader("robot_description")
{
  kinematic_model = robot_model_loader.getModel();

  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup("arm");
  const moveit::core::LinkModel *link_model = kinematic_state->getLinkModel("hand_link");
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

  Eigen::Translation3d cur_trans(current_pose.translation());
  Eigen::Quaterniond cur_quat(current_pose.rotation());
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
  // for now, just move all the way to the mouth in a single step
  Eigen::Affine3d pseudo_end_pose = target_trans * target_quat;
  return pseudo_end_pose;
}

void
JacobianController::move_to_target_pose(const Eigen::Affine3d &target_pose)
{
  Eigen::Translation3d cur_trans(current_pose.translation());
  Eigen::Quaterniond cur_quat(current_pose.rotation());
  Eigen::Translation3d target_trans(target_pose.translation());
  Eigen::Quaterniond target_quat(target_pose.rotation());
  Eigen::Translation3d trans_diff = cur_trans.inverse() * target_trans;
  Eigen::Quaterniond rot_diff =  cur_quat.inverse() * target_quat;
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  
  double trans_dist = distance(cur_trans, target_trans);
  Eigen::AngleAxisd rot_axis_angle(rot_diff);
  double rot_angle = rot_axis_angle.angle();
  Eigen::Vector3d rot_axis = rot_axis_angle.axis();
  std::cout << trans_dist << " is the translation distance. which is bigger than " << TRANS_STEP_SIZE << std::endl;
  if (trans_dist > TRANS_STEP_SIZE)
  {
    double step_scale = TRANS_STEP_SIZE / trans_dist;
    trans_dist = trans_dist * step_scale;
    rot_angle = rot_angle * step_scale;
    std::cout << "Translation was larger than TRANS_STEP_SIZE, so only going " << step_scale << " of the waythere";
  }
  
  std::cout << rot_angle << " is the rotation distance. which is bigger than " << TRANS_STEP_SIZE << std::endl;
  if (rot_angle > ANGLE_STEP_SIZE)
  {
    double angle_step_scale = ANGLE_STEP_SIZE / rot_angle;
    trans_dist = trans_dist * angle_step_scale;
    rot_angle = rot_angle * angle_step_scale;
    std::cout << "Rotation was larger than ANGLE_STEP_SIZE, so only going " << angle_step_scale << " of the waythere";
  }
  
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;
  const moveit::core::LinkModel *link_model = kinematic_state->getLinkModel("hand_link");
  kinematic_state->getJacobian(joint_model_group,
    link_model,
    reference_point_position,
    jacobian);

  Eigen::Matrix<double, 6, 1> target_delta;
  target_delta << target_pose.translation(), rot_angle * rot_axis;

  //https://eigen.tuxfamily.org/dox/group__LeastSquares.html
  Eigen::VectorXd joint_delta = (jacobian.transpose() * jacobian).ldlt().solve(jacobian.transpose() * target_delta);
  std::cout << "heading to " << joint_delta << std::endl;
  return;
}
