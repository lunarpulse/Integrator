#include "stm32f407xx.h"

#define SERVO_PIN 3 //GPIO PWM available

//TODO: this part is the motor state. need to move it to motor dedicated one.
//making a motor library? implementing servo library 
// the possible states of the state-machine
typedef enum {  NONE, GOT_N, GOT_D, GOT_I, GOT_S, GOT_M, GOT_F, GOT_R, GOT_P, GOT_U } States_t;
//GOT_D can be a direction indicator
//MOTOR status enums
typedef enum {STOP, RESTART, RUN} motorStates; //how about CL, ACL ?