#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
#include "zf_common_headfile.h"

extern double v;
void MotorInit (void);
void MotorCtrl3W(int motora,int motorb,int motorc);
void encoder_init ();

#endif /* CODE_MOTOR_H_ */
