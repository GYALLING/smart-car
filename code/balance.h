/*
 * pid.h
 *
 *  Created on: 2023Äê3ÔÂ31ÈÕ
 *      Author: ¹¢×ÏÑô
 */

#ifndef USER_BALANCE_H_
#define USER_BALANCE_H_
#include"zf_common_headfile.h"
extern float Roll_zero;//;
extern float Pitch_zero;
extern float Yaw_zero;

extern float PWM_A,PWM_yaw,PWM_C;
extern float PWMa_accel,PWMyaw_accel,PWMc_accel;
extern float PWM_A_jiaodu,PWM_yaw_jiaodu,PWM_C_jiaodu;
extern short Encoder_a,Encoder_b,Encoder_c;
extern float X_Velocity_kp,X_Velocity_ki,X_Velocity_kd;
extern float jiaosudu_kp,jiaosudu_ki,jiaosudu_kd;
extern float jiaodu_kp,jiaodu_ki,jiaodu_kd;
extern float C_jiaodu_kp,C_jiaodu_ki,C_jiaodu_kd;
extern float C_jiaosudu_kp,C_jiaosudu_ki,C_jiaosudu_kd;
extern float C_Velocity_KP,C_Velocity_KI,C_Velocity_KD;
extern float yaw_jiaosudu_kp,yaw_jiaosudu_ki,yaw_jiaosudu_kd;
extern float yaw_jiaodu_kp,yaw_jiaodu_ki,yaw_jiaodu_kd;
extern float yaw_sudu_kp,yaw_sudu_ki,yaw_sudu_kd;
extern float Y_Balance_KP,Y_Balance_KI,Y_Balance_KD;
extern int move_distance;
extern double error1,Bias1,Last_Bias1;

float jiaojiasudu_A_zengliang(float Gyro, float Zero);
float jiaojiasudu_A(float Gyro,float Zero);
float jiaojiasudu_C_zengliang(float Gyro, float Zero);
float jiaojiasudu_C(float Gyro,float Zero);
float jiaojiasudu_yaw(float Gyro,float Zero);
float Velocity_Control_A(int encoder);
float Velocity_Control_yaw(int encoder);
float Velocity_Control_C(int encoder);
float jiaodu_A(float Angle,float Angle_Zero);
float jiaodu_yaw(float Angle,float Angle_Zero);
float jiaodu_C(float Angle,float Angle_Zero);
float balance_UP_C(float Angle,float Mechanical_balance,float Gyro);
int Velocity(int encoder);
//float smoothUpdate(float target_yaw_last,float smoothFactor);
void smoothUpdate(float *target_yaw,float *target_yaw_last,float smoothFactor);
#endif /* USER_BALANCE_H_ */
