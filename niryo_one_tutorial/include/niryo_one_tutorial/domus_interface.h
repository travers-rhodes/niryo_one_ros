//
// This class connects to DOMUS via a serial connection 
// and can be used to send target angles for DOMUS to move to
//
#include <serial/serial.h>
#include <math.h>

#ifndef DOMUS_INTERFACE_H_
#define DOMUS_INTERFACE_H_
class DomusInterface
{
  public:
    DomusInterface();
    virtual void InitializeConnection();
    virtual void SendTargetAngles(const std::vector<double> &joint_angles);
  private:
    serial::Serial ser;
};
#endif
