#include "niryo_one_tutorial/domus_interface.h"
#include "ros/ros.h"

const uint8_t REQUEST_JOINT_ANGLES = 136;

std::vector<double> max_joint_angles{ 2.7, 0 + 0.64, 2.1 - 1.35, 2.7, 2.4, 2.5 };
std::vector<double> min_joint_angles{ -2.7, -2.3 + 0.64, 0 - 1.35, -2.7, -2.4, -2.5 };


DomusInterface::DomusInterface()
{
}

void
DomusInterface::InitializeConnection()
{
  // https://github.com/ros-drivers/um6/blob/indigo-devel/src/main.cpp
  ser.setPort("/dev/ttyACM0");
  ser.setBaudrate(115200);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);

  try
  {
    ser.open();
  }
  catch(const serial::IOException& e)
  {
    ROS_WARN("Unable to connect to port.");
  }
}

void
DomusInterface::SendTargetAngles(const std::vector<double> &joint_angles)
{
  for (int i = 0; i < joint_angles.size(); i++) {
    if (joint_angles[i] > max_joint_angles[i] || joint_angles[i] < min_joint_angles[i]) 
    {
      ROS_ERROR_STREAM("The requested joint " << i << " was " << joint_angles[i] << " which is past the joint limits.");
      return;
    }
  }
  ros::Rate r(100);
  // OpCode + 6 angles * 2 bytes
  uint8_t command[13];
  ROS_WARN_STREAM("I was asked to go to "<<joint_angles[0]
   <<","<<joint_angles[1]
   <<","<<joint_angles[2]
   <<","<<joint_angles[3]
   <<","<<joint_angles[4]
   <<","<<joint_angles[5]
   <<std::endl);
  
  command[0] = REQUEST_JOINT_ANGLES;
  uint16_t joint_temp;
  for (size_t joint_index = 0; joint_index < 6; joint_index++)
  {
    ROS_WARN("adding to my command");
    // let's say joint_angles ranges from -2pi to 2pi just to be extra safe
    // so, we'll map that via [-2pi, 2pi] -> [0, 65535]
    // giving us
    joint_temp = (joint_angles[joint_index] + 2.0 * M_PI)/(4.0 * M_PI) * 65535;
    ROS_WARN("getting there");
    // big byte
    command[joint_index * 2 + 1] = joint_temp >> 8;
    ROS_WARN("almost there");
    // small byte
    command[joint_index * 2 + 2] = joint_temp & 0xff;
  }
  ROS_WARN_STREAM("got my command ready"<<unsigned(command[0])
   <<","<<unsigned(command[11])
   <<","<<unsigned(command[12])
   <<std::endl);

  // OpCode + 6 angles * 2 bytes
  ser.write(command, 13);
  ser.flush();
}
