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
