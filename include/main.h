#ifndef _MIKUBOT_MAIN_H_
#define _MIKUBOT_MAIN_H_

#define PROS_USE_SIMPLE_NAMES 
#define PROS_USE_LITERALS 

extern "C" {
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
}

#endif
