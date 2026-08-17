#include "constrain.h"

float constrain_float(float amt, float low, float high)             //float类型限幅
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
int constrain_int(int amt, int low, int high)                       //int类型限幅
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
short constrain_short(short amt, short low, short high)             //shortl类型限幅
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
double constrain_double(double amt, double low, double high)             //shortl类型限幅
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
