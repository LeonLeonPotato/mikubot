#ifndef _PROS_ROBOT_H_
#define _PROS_ROBOT_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

#include "api.h"

namespace robot {
    double x, velocity_x, acceleration_x;
    double y, velocity_y, acceleration_y;
    double theta, angular_velocity, angular_acceleration;
    
    double left_set_velocity;
    double right_set_velocity;
}

#endif