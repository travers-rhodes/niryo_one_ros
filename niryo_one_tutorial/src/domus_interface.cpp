#include "niryo_one_tutorial/domus_interface.h"
#include "ros/ros.h"

const uint8_t REQUEST_JOINT_ANGLES = 136;


DomusInterface::DomusInterface() : ser()
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
  ros::Rate r(100);
  // OpCode + 6 angles * 2 bytes
  uint8_t command[13];
  
  command[0] = REQUEST_JOINT_ANGLES;
  uint16_t joint_temp;
  for (size_t joint_index = 0; joint_index < 6; joint_index++)
  {
    ROS_WARN("adding to my command");
    // let's say joint_angles ranges from -2pi to 2pi just to be extra safe
    // so, we'll map that via [-2pi, 2pi] -> [0, 65535]
    // giving us
    joint_temp = (joint_angles[joint_index] + 2.0 * M_PI)/(2.0 * M_PI) * 65535;
    ROS_WARN("getting there");
    // big byte
    command[joint_index * 2 + 1] = joint_temp >> 8;
    ROS_WARN("almost there");
    // small byte
    command[joint_index * 2 + 2] = joint_temp & 0xff;
  }
  ROS_WARN("got my command ready");

  // OpCode + 6 angles * 2 bytes
  ser.write(command, 13);
  ser.flush();
}
