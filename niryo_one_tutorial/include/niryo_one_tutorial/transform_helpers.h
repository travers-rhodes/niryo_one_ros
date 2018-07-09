#include <Eigen/Dense>
#include <Eigen/Geometry>

double quat_distance(const Eigen::Quaterniond &cur_quat, const Eigen::Quaterniond &end_quat);
double distance(const Eigen::Translation3d &cur_trans, const Eigen::Translation3d &end_trans);
