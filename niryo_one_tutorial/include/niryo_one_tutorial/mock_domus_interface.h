//
// This trivial version of domus_interface doesn't actually connect to the robot,
// so it's helpful to inject it in place of domus_interface 
// in times when you aren't connected to a physical robot
//
#include <niryo_one_tutorial/domus_interface.h>

#ifndef MOCK_DOMUS_INTERFACE_H_
#define MOCK_DOMUS_INTERFACE_H_
class MockDomusInterface : public DomusInterface 
{
  public:
    void InitializeConnection();
    void SendTargetAngles(const std::vector<double> &joint_angles);
};
#endif
