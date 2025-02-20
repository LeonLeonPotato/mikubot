#pragma once

#include "subsystems/base_system.h" // IWYU pragma: export
#include "subsystems/impl/conveyor.h" // IWYU pragma: export
#include "subsystems/impl/wallmech.h" // IWYU pragma: export
#include "subsystems/impl/driving.h" // IWYU pragma: export
#include "subsystems/impl/goalrush.h" // IWYU pragma: export
#include "subsystems/impl/clamp.h" // IWYU pragma: export

namespace subsystems {
    extern std::vector<subsystems::Subsystem*> subsystems;
} // namespace subsystems