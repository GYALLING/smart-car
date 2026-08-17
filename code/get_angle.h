
/*
 * get_angle.h
 *
 *  Created on: 2025年3月4日
 *      Author: HP
 */

#ifndef CODE_GET_ANGLE_H_
#define CODE_GET_ANGLE_H_
#include "zf_common_headfile.h"
#define XO 0.001064225  //PI/180/16.4f
#define ZO 0.0002442598   //4090的倒数
//#define ZO 0.000244140625   //4096的倒数
#define alpha  0.35   //一阶低通滤波值
//-------------------------------越野-----------------------------------
//#define XO 0.001064225  //PI/180/16.4f
//#define ZO 0.0002442598   //4090的倒数
//#define alpha  0.35   //一阶低通滤波值
struct icm{
    float icm_ax;//角度
  float icm_ay;
  float icm_az;
    float icm_gx;//角速度
    float icm_gy;
    float icm_gz;
};
extern void attitude_solution();
extern void getimu_value();
extern void zero();//零飘
extern float angle_speed;//角速度
extern struct icm att,att_2;
extern float yaw;//角度
extern struct icm att,att_2;
//-----------------------------------------------------------------------
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void Zero_Attitude();
void Attitude_get();
void Data_Filter();
float invSqrt(float x);
void Data_Filter1();
float invSqrt(float x);
extern float q0 , q1 , q2 , q3,q00,q11,q22,q33,q44;
extern float yaw,pitch,roll;//静态姿态
extern float Yaw,Yaw1,Yaw2,Pitch,Pitch1,Roll,Roll1;
extern float X,Y,Z;
extern float a,j_z,Kp ,Ki,Kp2 ,Ki2;
extern float Gyro[3];
extern float acc_lpf[3];
extern float gyr_lpf[3];
extern float gyrozero[3];
extern short acc[3];
extern short gyr[3];
extern float gyrset[3];
typedef struct
{
    float X,Y,Z;
}XYZ;

extern XYZ ACC;
extern XYZ GYRO;
extern XYZ ACC_Real;
extern XYZ GYRO_Real;


#endif /* CODE_GET_ANGLE_H_ */

