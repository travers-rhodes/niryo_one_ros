#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <niryo_one_tutorial/domus_interface.h>
#include <niryo_one_tutorial/transform_helpers.h>

class JacobianController
{
  public:
    JacobianController(double trans_step_size_meters, DomusInterface* domus_interface, ros::NodeHandle* n);
    void make_step_to_target_pose(const geometry_msgs::Pose &target_pose);

  private:
    void move_to_target_pose(const Eigen::Affine3d &target_pose);
    Eigen::Affine3d get_pseudo_end_pose(Eigen::Translation3d, Eigen::Quaterniond);
    Eigen::MatrixXd get_cylindrical_jacobian();
    void publish_robot_state();

    Eigen::Affine3d current_pose_;
    robot_state::RobotStatePtr kinematic_state_;
    const robot_state::JointModelGroup* joint_model_group_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    ros::Publisher joint_pub_;
    ros::Publisher dist_pub_;
    sensor_msgs::JointState joint_state_;
    DomusInterface* domus_interface_;
    float _trans_step_size_meters;
};
