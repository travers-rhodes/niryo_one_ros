#include <niryo_one_tutorial/jacobian_controller.h>

const double TRANS_EPSILON = 0.01;
const double QUAT_EPSILON = 0.01;
const double ANGLE_STEP_SIZE = 0.1;
const double TRANS_STEP_SIZE = 0.01;
const double MAX_JOINT_STEP = 0.1;

//constructor
JacobianController::JacobianController(DomusInterface* domus_interface, ros::NodeHandle* n) 
  : robot_model_loader_("robot_description")
{
  kinematic_model_ = robot_model_loader_.getModel();
  domus_interface_ = domus_interface;
  domus_interface_->InitializeConnection();

  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kinematic_model_->getJointModelGroup("arm");

  std::vector<double> initial_joint_values { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  domus_interface->SendTargetAngles(initial_joint_values);
  kinematic_state_->setJointGroupPositions(joint_model_group_, initial_joint_values);  
  current_pose_ = kinematic_state_->getGlobalLinkTransform("hand_link");

  // set up joint publishing
  joint_pub_ = n->advertise<sensor_msgs::JointState>("/domus/robot/joint_states", 1);
  dist_pub_ = n->advertise<std_msgs::Float64>("/distance_to_target", 1);
  std::cout << "Sleeping for 5 seconds to get to initial position";
  ros::Duration(5).sleep();
}

void
JacobianController::make_step_to_target_pose(const geometry_msgs::Pose &target_pose)
{
  Eigen::Quaterniond target_quat(target_pose.orientation.w,
                              target_pose.orientation.x,
                              target_pose.orientation.y, 
                              target_pose.orientation.z);

  // because this is coming in from a ROS topic, they might not have properly normalized it when typing
  // in the desired quaternion...Travers
  target_quat.normalize();
  Eigen::Translation3d target_trans(target_pose.position.x,
                                    target_pose.position.y,
                                    target_pose.position.z);

  Eigen::Translation3d cur_trans(current_pose_.translation());
  Eigen::Quaterniond cur_quat(current_pose_.rotation());
  double quat_dist = quat_distance(cur_quat, target_quat);
  double trans_dist = distance(cur_trans, target_trans);
  //std::cout << trans_dist << " is the translation distance. " << quat_dist << " is the quat dist" << std::endl;
  //std::cout << target_quat.w() << "," <<  target_quat.vec() << " is the target quat. " << cur_quat.w() << "," << target_quat.vec() << " is the current quat" << std::endl;
  std_msgs::Float64 msg;
  if (trans_dist < TRANS_EPSILON &&
      quat_dist < QUAT_EPSILON)
  {
    //no need to move
    //publish current pose (we say we are there, so publish a distance of 0)
    msg.data = 0.0;
    dist_pub_.publish(msg); 
    return;
  }
  msg.data = 1.0;
  // we aren't there yet, so we just publish a distance of 1
  dist_pub_.publish(msg); 
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
  Eigen::Translation3d cur_trans(current_pose_.translation());
  Eigen::Quaterniond cur_quat(current_pose_.rotation());
  Eigen::Translation3d target_trans(target_pose.translation());
  Eigen::Quaterniond target_quat(target_pose.rotation());
  Eigen::Translation3d trans_matrix = cur_trans.inverse() * target_trans;
  Eigen::Vector3d trans_diff = trans_matrix.translation();
  Eigen::Quaterniond rot_diff =  target_quat * cur_quat.inverse();
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  
  double quat_dist = quat_distance(cur_quat, target_quat);
  std::cout << target_quat.w() << "," <<  target_quat.vec() << " is the target quat. " << cur_quat.w() << "," << target_quat.vec() << " is the current quat" << std::endl;
  std::cout << "Quat dist is now " << quat_dist << std::endl;
  double trans_dist = distance(cur_trans, target_trans);
  Eigen::AngleAxisd rot_axis_angle(rot_diff);
  double rot_angle = rot_axis_angle.angle();
  Eigen::Vector3d rot_axis = rot_axis_angle.axis();
  std::cout << trans_dist << " is the translation distance. trans step size is " << TRANS_STEP_SIZE << std::endl;
  // cylindrical_diff is of the form dR, dTheta, dZ
  // where R is sqrt(x^2 + y^2), theta is arctan2(y,x), and z is z
  Eigen::Vector3d cylindrical_diff = get_cylindrical_point_translation(cur_trans.translation(), target_trans.translation());

  if (trans_dist > TRANS_STEP_SIZE)
  {
    double step_scale = TRANS_STEP_SIZE / trans_dist;
    trans_diff = trans_diff * step_scale;
    rot_angle = rot_angle * step_scale;
    cylindrical_diff = cylindrical_diff * step_scale;
    std::cout << "Translation was larger than TRANS_STEP_SIZE, so only going " << step_scale << " of the waythere" << std::endl;
  }
  
  std::cout << rot_angle << " is the rotation distance. angle step size is" << ANGLE_STEP_SIZE << std::endl;
  std::cout << "The rotation axis is" << rot_axis << std::endl;
  if (rot_angle > ANGLE_STEP_SIZE)
  {
    double angle_step_scale = ANGLE_STEP_SIZE / rot_angle;
    trans_diff = trans_diff * angle_step_scale;
    rot_angle = rot_angle * angle_step_scale;
    cylindrical_diff = cylindrical_diff * angle_step_scale;
    std::cout << "Rotation was larger than ANGLE_STEP_SIZE, so only going " << angle_step_scale << " of the waythere" << std::endl;
  }
  
  Eigen::MatrixXd jacobian = get_cylindrical_jacobian();

  std::cout << "jacobian was computed to be" << jacobian << std::endl; 

  // get a single column vector of the translation and rotation we want to achieve
  Eigen::Matrix<double, 6, 1> target_delta;
  target_delta << cylindrical_diff, rot_angle * rot_axis;

  //https://eigen.tuxfamily.org/dox/group__LeastSquares.html
  // with the extra addition that we use a tiny regularization term to reduce problems due to singularities, so we're solving (J^T J + lambda * I)^-1 J^T Y
  double lambda = 0.1;
  Eigen::VectorXd joint_delta = (jacobian.transpose() * jacobian + (lambda * Eigen::MatrixXd::Identity(6,6))).ldlt().solve(jacobian.transpose() * target_delta);
  std::cout << "heading to " << joint_delta << std::endl;
  std::vector<double> current_joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_, current_joint_values);
  // ensure that no joint rotation is larger than MAX_JOINT_STEP at any given time
  for(int i = 0; i < 6; i++)
  {
    double cur_joint_step = std::abs(joint_delta[i]);
    if (cur_joint_step > MAX_JOINT_STEP)
    {
      double scale = MAX_JOINT_STEP / cur_joint_step;
      std::cout << "We're scaling now, by a factor of " << scale << std::endl;
      for (int j = 0; j < 6; j++)
      {
        joint_delta[j] = joint_delta[j] * scale;
      }
    }
  }
  std::vector<double> new_joint_values(6);
  for(std::size_t i = 0; i < 6; ++i)
  {
    new_joint_values[i] = joint_delta(i) + current_joint_values[i];
  }
  domus_interface_->SendTargetAngles(new_joint_values);
  ros::Duration(0.1).sleep();
  kinematic_state_->setJointGroupPositions(joint_model_group_, new_joint_values);  
  current_pose_ = kinematic_state_->getGlobalLinkTransform("hand_link");
  publish_robot_state();
  return;
}

// The jacobian in cylindrical coordinates
Eigen::MatrixXd
JacobianController::get_cylindrical_jacobian()
{
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;
  const moveit::core::LinkModel *link_model = kinematic_state_->getLinkModel("hand_link");
  kinematic_state_->getJacobian(joint_model_group_,
    link_model,
    reference_point_position,
    jacobian);
  std::cout << "rect_jacob "<< jacobian << std::endl; 

  Eigen::Vector3d cur_trans(current_pose_.translation());
  Eigen::Matrix<double,6,6> rect_to_cyl_jacob = compute_jacob_from_rect_to_cyl(cur_trans);
  std::cout << "rect_to_cyl_jacob" << rect_to_cyl_jacob << std::endl; 
  
  Eigen::Matrix<double,6,6> cyl_jacobian = rect_to_cyl_jacob * jacobian;
  return cyl_jacobian;
}

void
JacobianController::publish_robot_state()
{
  const std::vector<std::string> &joint_names = joint_model_group_->getJointModelNames();
  std::vector<double> joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
  joint_state_.header.stamp = ros::Time::now();
  joint_state_.name.resize(joint_values.size());
  joint_state_.position.resize(joint_values.size());
  for (int i = 0; i < joint_values.size(); i++) {
    joint_state_.name[i] = joint_names[i];
    joint_state_.position[i] = joint_values[i];
  }
  joint_pub_.publish(joint_state_);
  return;
}


