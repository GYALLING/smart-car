#ifndef USER_CONSTRAIN_H_
#define USER_CONSTRAIN_H_
#include"zf_common_headfile.h"

float constrain_float(float amt, float low, float high);
int constrain_int(int amt, int low, int high);
short constrain_short(short amt, short low, short high);
double constrain_double(double amt, double low, double high);

#endif
