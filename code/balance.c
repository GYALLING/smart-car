/*
 * pid.c
 *
 *  Created on: 2023年3月31日
 *      Author: 耿紫阳
 */
/// pid参数初始化函数
#include <balance.h>
#include "zf_common_headfile.h"


float Pitch_zero=0.0;
float Roll_zero=0.0;
float Yaw_zero=0.0;



float PWM_A,PWM_yaw=0,PWM_C,PWMa_accel=0,PWMyaw_accel=0,PWMc_accel=0;
float PWM_A_jiaodu=0,PWM_yaw_jiaodu=0,PWM_C_jiaodu=0;
short Encoder_a,Encoder_b,Encoder_c;



float jiaosudu_kp=150,jiaosudu_ki=0.11,jiaosudu_kd=25;
//float jiaodu_kp=-80,jiaodu_ki=0,jiaodu_kd=-50;
//float X_Velocity_kp=-3.6,X_Velocity_ki=0,X_Velocity_kd=0;

float C_jiaosudu_kp=0,C_jiaosudu_ki=0,C_jiaosudu_kd=0;
//float C_jiaodu_kp=0,C_jiaodu_ki=0,C_jiaodu_kd=0;
//float C_Velocity_KP=0,C_Velocity_KI=0,C_Velocity_KD=0;


//float jiaosudu_zengliang_kp=0.5,jiaosudu_zengliang_ki=0.06,jiaosudu_zengliang_kd=0.2;//12.3v i=0.06
//float jiaodu_kp=-50,jiaodu_ki=-0.001,jiaodu_kd=-0;//p=-40
//float X_Velocity_kp=-0.1,X_Velocity_ki=0,X_Velocity_kd=0;


//float C_jiaosudu_zengliang_kp=0,C_jiaosudu_zengliang_ki=0,C_jiaosudu_zengliang_kd=0;
//float C_jiaodu_kp=0,C_jiaodu_ki=0,C_jiaodu_kd=-0;
//float C_Velocity_KP=0,C_Velocity_KI=0,C_Velocity_KD=-0;

//float jiaosudu_zengliang_kp=0.5,jiaosudu_zengliang_ki=0.065,jiaosudu_zengliang_kd=0.3;// 二队 0.5 0.065 0.25  1.0版12.2v  前面自己后面参考数值  0.5 0.065 0.2
//float jiaodu_kp=-52,jiaodu_ki=-0.001,jiaodu_kd=-0.03;//-52 -0.001 -0.01   -51 -0.1 -0.05
//float X_Velocity_kp=-0.25,X_Velocity_ki=0,X_Velocity_kd=0;//-0.11  -0.21



//float C_jiaosudu_zengliang_kp=1.6,C_jiaosudu_zengliang_ki=0.038,C_jiaosudu_zengliang_kd=21;//二队1.6 0.035 4 2.0版 0.035  20
//float C_jiaodu_kp=-80,C_jiaodu_ki=0,C_jiaodu_kd=0;//-80
//float C_Velocity_KP=7.5,C_Velocity_KI=0.02,C_Velocity_KD=2;//7.5  2

/*
float jiaosudu_zengliang_kp=0.5,jiaosudu_zengliang_ki=0.065,jiaosudu_zengliang_kd=0.25;//12.3v i=0.06 一队
float jiaodu_kp=-52,jiaodu_ki=-0.001,jiaodu_kd=-0.01;//p=-40
float X_Velocity_kp=-0.26,X_Velocity_ki=0,X_Velocity_kd=0;

float C_jiaosudu_zengliang_kp=1.6,C_jiaosudu_zengliang_ki=0.035,C_jiaosudu_zengliang_kd=21;// 一队 -1.5 -0.335 -3
float C_jiaodu_kp=-58,C_jiaodu_ki=0,C_jiaodu_kd=0;//-55   -1.1
float C_Velocity_KP=8,C_Velocity_KI=0,C_Velocity_KD=0;//12.5

float yaw_jiaosudu_kp=-1.55,yaw_jiaosudu_ki=0,yaw_jiaosudu_kd=-0.3;//-1.95 -0.3、||-1.55 -0.3
float yaw_jiaodu_kp=72,yaw_jiaodu_ki=0,yaw_jiaodu_kd=16;//52 6   62 11  ||68 12
float yaw_sudu_kp=0.0,yaw_sudu_ki=0,yaw_sudu_kd=0;//16
*/

float jiaosudu_zengliang_kp=0.5,jiaosudu_zengliang_ki=0.065,jiaosudu_zengliang_kd=0.3;// 二队 0.5 0.065 0.25  1.0版12.2v  前面自己后面参考数值  0.5 0.065 0.2
float jiaodu_kp=-52,jiaodu_ki=-0.001,jiaodu_kd=-0.03;//-52 -0.001 -0.01   -51 -0.1 -0.05
float X_Velocity_kp=-0.21,X_Velocity_ki=0,X_Velocity_kd=0;//-0.11  -0.21

float C_jiaosudu_zengliang_kp=1.6,C_jiaosudu_zengliang_ki=0.038,C_jiaosudu_zengliang_kd=21;//二队1.6 0.035 4 2.0版 0.035  20
float C_jiaodu_kp=-58,C_jiaodu_ki=0,C_jiaodu_kd=-1;//-80
float C_Velocity_KP=7.5,C_Velocity_KI=0.02,C_Velocity_KD=2;//7.5  2

float yaw_jiaosudu_kp=-1.55,yaw_jiaosudu_ki=0,yaw_jiaosudu_kd=-0.3;//-1.95 -0.3、||-1.55 -0.3
float yaw_jiaodu_kp=65,yaw_jiaodu_ki=0,yaw_jiaodu_kd=16;//52 6   62 11  ||68 12
float yaw_sudu_kp=0.0,yaw_sudu_ki=0,yaw_sudu_kd=0;//16



//float C_jiaosudu_zengliang_kp=1.8,C_jiaosudu_zengliang_ki=0.04,C_jiaosudu_zengliang_kd=4;//-0.9 -0.025 2.0版
//float C_jiaodu_kp=-68,C_jiaodu_ki=0,C_jiaodu_kd=-1;//-40
//float C_Velocity_KP=17,C_Velocity_KI=0,C_Velocity_KD=1.5;//-7.5



//float yaw_jiaosudu_kp=-3.0,yaw_jiaosudu_ki=0,yaw_jiaosudu_kd=0;//0.001//电磁最新-2.25
//float yaw_jiaodu_kp=,yaw_jiaodu_ki=0,yaw_jiaodu_kd=0;//52  8
//float yaw_sudu_kp=0.0,yaw_sudu_ki=0,yaw_sudu_kd=0;//16



//float yaw_jiaosudu_kp=-6.05,yaw_jiaosudu_ki=0,yaw_jiaosudu_kd=-0.7;//0.001//电磁最新
//float yaw_jiaodu_kp=25,yaw_jiaodu_ki=0,yaw_jiaodu_kd=7;
//float yaw_sudu_kp=0.0,yaw_sudu_ki=0,yaw_sudu_kd=0;//16




//float yaw_jiaosudu_kp=-2.05,yaw_jiaosudu_ki=0,yaw_jiaosudu_kd=-0.8;//0.001//电磁最新-2.25
//float yaw_jiaodu_kp=50,yaw_jiaodu_ki=0,yaw_jiaodu_kd=8;//52  8
//float yaw_sudu_kp=0.0,yaw_sudu_ki=0,yaw_sudu_kd=0;//16

//


int move_distance=0;
//X前后，Y左右；
double error1,Bias1,Last_Bias1;

/*******************************************************************
函数功能：直立PD控制
入口参数：角度、机械平衡角度（机械中值）、角速度
返回  值：直立控制PWM
作    者：张巧龙
******************************************************************/
float balance_C_KP=250,balance_C_KD=2.5;
float velocity_KP=0,velocity_KI=0;
float balance_UP_C(float Angle,float Mechanical_balance,float Gyro)
{
   float Bias;//角度误差
   float balance;//直立环计算出来的电机控制pwm
   Bias=Angle-Mechanical_balance;
   //===求出平衡的角度中值和机械相关
   balance=balance_C_KP*Bias+balance_C_KD*Gyro;
   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
   return balance;
}
/**************************************
入口参数：电机编码器的值
返回  值：速度控制PWM
作    者：张巧龙
**************************************/
int Velocity(int encoder)
{
    static float Velocity,Encoder_Least,Encoder,Movement;
    static float Encoder_Integral;
   //=============速度PI控制器=======================//
    Encoder_Least =encoder-0;
    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
    Encoder *= 0.7;          //===一阶低通滤波器
    Encoder += Encoder_Least*0.3;   //===一阶低通滤波器
    Encoder_Integral +=Encoder; //===积分出位移 积分时间：10ms
    if(Encoder_Integral>10000)    Encoder_Integral=10000;
    //===积分限幅
    if(Encoder_Integral<-10000)    Encoder_Integral=-10000;
    //===积分限幅
    Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;
    //===速度控制
    //if(pitch<-40||pitch>40)   Encoder_Integral=0;
    //===电机关闭后清除积分
    return Velocity;
}


float jiaojiasudu_A_zengliang(float Gyro, float Zero)
{
    static float PWM=0;
    float _pwm;
    static double error = 0, Last_error = 0, Last_last_error = 0;
    //if (fabs(Gyro) < 3) Gyro = 0;
    error = Gyro - Zero;
    _pwm = jiaosudu_zengliang_kp * (error - Last_error) + jiaosudu_zengliang_ki * (error)+jiaosudu_zengliang_kd * (error - 2 * Last_error + Last_last_error);
    PWM += _pwm;
    Last_last_error = Last_error;
    Last_error = error;
    PWM = constrain_float(PWM, -PWM_DUTY_MAX, PWM_DUTY_MAX);

    return PWM;
}
float jiaojiasudu_A(float Gyro,float Zero){
    float PWM;
    static double Last_Bias,Bias;
    if(fabs(Gyro)<1) Gyro=0;
    Bias=Gyro-Zero;                                           //获取偏差
    error1+=Bias;                                                         //偏差累积
    error1 = constrain_float(error1, -3000, 3000);                            //积分限幅
    PWM=jiaosudu_kp*Bias + jiaosudu_ki*error1+jiaosudu_kd*(Bias-Last_Bias);   //获取最终数值
    PWM = constrain_float(PWM, -PWM_DUTY_MAX, PWM_DUTY_MAX);                            //输出限幅
    Last_Bias=Bias;
    return PWM;
}
float jiaodu_A(float Angle,float Angle_Zero)
{
//    static unsigned int n;                         //计数，用来改变零点
    float PWM,Bias;
    static double error,Last_Bias;
    Bias=Angle-Angle_Zero;                                              //获取偏差
    error+=Bias;                                                         //偏差累积
    error = constrain_float(error, -3000, 3000);                            //积分限幅
    PWM=jiaodu_kp*Bias +jiaodu_ki*error+jiaodu_kd*(Bias-Last_Bias);   //获取最终数值
    Last_Bias=Bias;
    return PWM;
}
/**
 * A电机速度环
 */
float Velocity_Control_A(int encoder)
{
    static float Encoder,Encoder_Integral;
        float Velocity,Encoder_Least;
        Encoder_Least = (float)encoder;
//Encoder_Least是这次误差
//    Encoder_Least = (float)encoder;                                                   //速度滤波
    Encoder *= 0.7;                                                              //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                                                //一阶低通滤波器
    Encoder_Integral += Encoder;                                               //积分出位移,Encoder就是error（偏差值）
    Encoder_Integral = constrain_float(Encoder_Integral, -3500, 3500);        //积分限幅
    Velocity = Encoder * X_Velocity_kp/100 + Encoder_Integral * X_Velocity_ki/100+X_Velocity_kd/100*(Encoder-Encoder_Least); //获取最终数值
    Velocity = constrain_float(Velocity, -14, 14);
    return Velocity;
}
/**
 * C电机速度环
 */
float jiaojiasudu_C_zengliang(float Gyro, float Zero)
{
    static float PWM=0;
    float _pwm;
    static double error = 0, Last_error = 0, Last_last_error = 0;
    if (fabs(Gyro) < 1) Gyro = 0;
    error = Gyro - Zero;
    _pwm = C_jiaosudu_zengliang_kp * (error - Last_error) + C_jiaosudu_zengliang_ki * (error)+C_jiaosudu_zengliang_kd * (error - 2 * Last_error + Last_last_error);
    PWM += _pwm;
    Last_last_error = Last_error;
    Last_error = error;
    return PWM;
}
float jiaojiasudu_C(float Gyro,float Zero){
    double PWM,Bias;
    static double error,Last_Bias;
    if(fabs(Gyro)<1) Gyro=0;
    Bias=Gyro-Zero;                                           //获取偏差
    error+=Bias;                                                         //偏差累积
    error = constrain_float(error, -1000, 1000);                            //积分限幅
    PWM=C_jiaosudu_kp*Bias + C_jiaosudu_ki*error+C_jiaosudu_kd*(Bias-Last_Bias);   //获取最终数值
    PWM = constrain_float(PWM, -3500, 3500);                            //输出限幅
    Last_Bias=Bias;
    return PWM;
}
float Velocity_Control_C(int encoder)
{
    static float Encoder,Encoder_Integral;
        float Velocity,Encoder_Least;
        Encoder_Least = (float)encoder;
    Encoder *= 0.7;                                                            //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                                              //一阶低通滤波器
    Encoder_Integral += Encoder - move_distance;                             //积分出位移
    Encoder_Integral = constrain_float(Encoder_Integral, -1000, 1000);        //积分限幅
    Velocity = Encoder * C_Velocity_KP/100 + Encoder_Integral * C_Velocity_KI/100+C_Velocity_KD/100*(Encoder-Encoder_Least);   //获取最终数值
    Velocity = constrain_float(Velocity, -4.5, 4.5);
    return Velocity;
}
float jiaodu_C(float Angle,float Angle_Zero)
{
//    static unsigned int n;                         //计数，用来改变零点
    double PWM,Bias;
    static double error,Last_Bias;
    Bias=Angle-Angle_Zero;                                              //获取偏差
    error+=Bias;                                                         //偏差累积
    error = constrain_float(error, -1000, 1000);                            //积分限幅
    PWM=C_jiaodu_kp*Bias +C_jiaodu_ki*error+C_jiaodu_kd*(Bias-Last_Bias);   //获取最终数值
    Last_Bias=Bias;
//    PWM = constrain_float(PWM, -3000, 3000);                            //输出限幅
    return PWM;
}
/**
 * 转向环，结合yaw
 */
float jiaojiasudu_yaw(float Gyro,float Zero){
    double PWM,Bias;
    static double error,Last_Bias;
    if(fabs(Gyro)<1) Gyro=0;
    Bias=Gyro-Zero;                                           //获取偏差
    error+=Bias;                                                         //偏差累积
    PWM=yaw_jiaosudu_kp*Bias + yaw_jiaosudu_ki*error+yaw_jiaosudu_kd*(Bias-Last_Bias);   //获取最终数值
    PWM = constrain_float(PWM, -10000, 10000);                            //输出限幅
    Last_Bias=Bias;
    return PWM;
}
float jiaodu_yaw(float Angle,float Angle_Zero)
{
//    static unsigned int n;                         //计数，用来改变零点
    double PWM,Bias;
    static double error,Last_Bias;
//    if(fabs(Angle)<=3) Angle=0;
    Bias=Angle-Angle_Zero;                                              //获取偏差
    error+=Bias;                                                         //偏差累积
    error = constrain_float(error, -3000, 3000);                            //积分限幅
    PWM=yaw_jiaodu_kp*Bias +yaw_jiaodu_ki*error+yaw_jiaodu_kd*(Bias-Last_Bias);   //获取最终数值
    Last_Bias=Bias;
    return PWM;
}
float Velocity_Control_yaw(int encoder)
{
    static float Encoder,Encoder_Integral;
        float Velocity,Encoder_Least;
        Encoder_Least = (float)encoder;                                   //速度滤波
    Encoder *= 0.7;                                                            //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                                              //一阶低通滤波器
    Encoder_Integral += Encoder ;                             //积分出位移
    Encoder_Integral = constrain_float(Encoder_Integral, -3000, 3000);        //积分限幅
    Velocity = Encoder * yaw_sudu_kp/100 + Encoder_Integral * yaw_sudu_ki/100+yaw_sudu_kd/100*(Encoder-Encoder_Least);   //获取最终数值
//    Roll_Zero = Roll_Zero1 - Move_distance/500;
    return Velocity;
}


//void smoothUpdate(float *target_yaw,float *target_yaw_last,float smoothFactor)
//{
//    if(abs(*target_yaw-*target_yaw_last)<10)
//    {
//        *target_yaw_last=*target_yaw;
//    }
//    else if(abs(*target_yaw-*target_yaw_last))
//    {
//        *target_yaw_last +=smoothFactor*(*target_yaw-*target_yaw_last);
//    }
////    else if(abs(*target_yaw-*target_yaw_last)>80)
////    {
////        if(*target_yaw-target_yaw_last>0)
////        {
////          *target_yaw_last=*target_yaw_last+1;
////        }
////        else if(*target_yaw-*target_yaw_last<0)
////        {
////            *target_yaw_last=*target_yaw_last-1;
////        }
////    }
//    *target_yaw_last=constrain_float(*target_yaw_last,-50,50);
//    *target_yaw=*target_yaw_last;
//}
void smoothUpdate(float *target_yaw, float *target_yaw_last, float smoothFactor)
{
    float delta = *target_yaw - *target_yaw_last;
    float max_step = 5.0f;  // 最大单步变化量（度/控制周期）
    // 小角度差：直接跟随
    if (fabsf(delta) < 10.0f)
    {
        *target_yaw_last = *target_yaw;
    }
    // 中角度差：比例平滑
    else if (fabsf(delta) < 80.0f)
    {
        *target_yaw_last += smoothFactor * delta;
    }
    // 大角度差：限速逼近
    else
    {
        float step = copysignf(max_step, delta);  // 保持方向
        *target_yaw_last += step;
    }

    // 软约束处理（避免阶跃）
    if (*target_yaw_last > 50.0f)
    {
        *target_yaw_last = 50.0f - 0.5f*(*target_yaw_last-50.0f);  // 边界缓冲
    }
    else if (*target_yaw_last < -50.0f)
    {
        *target_yaw_last = -50.0f - 0.5f*(*target_yaw_last+50.0f);
    }

    *target_yaw = *target_yaw_last;  // 同步输出
}



//float smoothUpdate(float target_yaw_last,float smoothFactor)
//{
//    if(abs(target_yaw-target_yaw_last)<10)
//    {
//        target_yaw_last=target_yaw;
//    }
//    else
//    {
//        target_yaw_last+=smoothFactor*(target_yaw-target_yaw_last);
//    }
//    target_yaw_last=constrain_float(target_yaw_last,-55,55);
//    return  target_yaw_last;
//
//}
