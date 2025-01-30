#include "subsystems.h"

std::vector<subsystems::Subsystem*> subsystems::subsystems = {
    new subsystems::Conveyor(),
    new subsystems::WallMech(),
    new subsystems::Driving(),
    new subsystems::Doinker(),
    new subsystems::Clamp()
};