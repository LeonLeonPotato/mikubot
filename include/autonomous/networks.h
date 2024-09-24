#pragma once

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

namespace networks {
class Network {
public:
    Network(void);
    ~Network(void);
    virtual void forward(void) = 0;
};
}