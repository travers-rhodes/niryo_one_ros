#include <niryo_one_tutorial/transform_helpers.h>

//Helper functions 
double quat_distance(const Eigen::Quaterniond &cur_quat, const Eigen::Quaterniond &end_quat)
{
  Eigen::Quaterniond diff =  cur_quat.inverse() * end_quat;
  Eigen::AngleAxisd rot_axis_angle(diff);
  double rot_angle = rot_axis_angle.angle();
  return rot_angle; 
}

double distance(const Eigen::Translation3d &cur_trans, const Eigen::Translation3d &end_trans)
{
  Eigen::Translation3d diff = cur_trans.inverse() * end_trans;
  return std::sqrt(std::pow(diff.x(), 2) + std::pow(diff.y(), 2) + std::pow(diff.z(),2));
}
