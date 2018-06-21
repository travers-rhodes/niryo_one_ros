#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class JacobianController
{
  public:
    JacobianController();
    void make_step_to_target_pose(const geometry_msgs::Pose &target_pose);
    void move_to_target_pose(const Eigen::Affine3d &target_pose);

  private:
    Eigen::Affine3d current_pose;
    unsigned long current_time;
    robot_state::RobotStatePtr kinematic_state;
    const robot_state::JointModelGroup* joint_model_group;
    Eigen::Affine3d get_pseudo_end_pose(Eigen::Translation3d, Eigen::Quaterniond);
    robot_model::RobotModelPtr kinematic_model;
    robot_model_loader::RobotModelLoader robot_model_loader;
};
