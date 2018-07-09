#include "niryo_one_tutorial/mock_domus_interface.h"

// override virtual methods with no-ops
void
MockDomusInterface::InitializeConnection()
{
}

void
MockDomusInterface::SendTargetAngles(const std::vector<double> &joint_angles)
{
}
