
#include"get_angle.h"
#include "zf_common_headfile.h"

XYZ ACC;
XYZ GYRO;
 XYZ ACC_Real;
 XYZ GYRO_Real;
 #define fullT 0.005f
#define Gyro_Gr     0.0010642




float Kp =3.0f;
float Ki=0.0001f;
float Kp2 =37.248f;
float Ki2 =0.1970f;
float X,Y,Z;
#define halfT 0.0025f
float q0 = 1.0, q1 = 0, q2 = 0, q3 = 0;

float exInt = 0, eyInt = 0, ezInt = 0;


float yaw,pitch,roll;
float Yaw,Yaw1,Yaw2,Pitch,Pitch1,Roll,Roll1;

float a = 0.06097561 ,j_z;

struct icm att,att_2;
float acc_lpf[3];
float gyr_lpf[3];
float Gyro[3];
short acc[3];
short gyr[3];
float gyrset[3];

float norm = 0;

float error_x = 0, error_y = 0, error_z = 0;
float q0temp,q1temp,q2temp,q3temp;
float posture_x =0, posture_y = 0, posture_z = 0;

float xErrorInt = 0, yErrorInt = 0, zErrorInt = 0;

float gx = 0,gy = 0,gz =0;
float ax =0 ,ay=0, az=0;

float Eorr = 0.35,yaw_last =0,yaw_u =0 ;

float angle_speed,yaw_speed[6] = {0,0,0,0,0,0};

int yaw1,yaw2,yaw3,road1;
//-----------------------------------------------------------------
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
        float norm;

        float q0temp,q1temp,q2temp,q3temp;
        float vx, vy, vz;

        float ex, ey, ez;

        gx=gx*Gyro_Gr;
        gy=gy*Gyro_Gr;
        gz=gz*Gyro_Gr;



        norm = sqrt(ax*ax + ay*ay + az*az);
        if(norm==0)return;

        ax = ax / norm;

        ay = ay / norm;

        az = az / norm;


        vx = 2*(q1*q3 - q0*q2);

        vy = 2*(q0*q1 + q2*q3);

        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;




        ex = (ay*vz - az*vy);

        ey = (az*vx - ax*vz);

        ez = (ax*vy - ay*vx);




        exInt = exInt + ex*Ki;

        eyInt = eyInt + ey*Ki;

        ezInt = ezInt + ez*Ki;





        gx = gx + Kp*ex + exInt;

        gy = gy + Kp*ey + eyInt;

        gz = gz + Kp*ez + ezInt;


        q0temp=q0;
        q1temp=q1;
        q2temp=q2;
        q3temp=q3;


        q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
        q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
        q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
        q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;



        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if(norm==0)return;

        q0 = q0 / norm;

        q1 = q1 / norm;

        q2 = q2 / norm;

        q3 = q3 / norm;
        Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;

}

void Attitude_get()
{
    Gyro[0]=gyr_lpf[0]-gyrset[0];
    Gyro[1]=gyr_lpf[1]-gyrset[1];
    Gyro[2]=gyr_lpf[2]-gyrset[2];

    IMUupdate(Gyro[0],Gyro[1],Gyro[2],acc_lpf[0],acc_lpf[1],acc_lpf[2]);

}

void Zero_Attitude()
{
  int i;
  for(i = 0;i < 1000;i ++)
  {
      imu660rb_get_acc();
      imu660rb_get_gyro();
      Data_Filter();

      gyrset[0]+=gyr_lpf[0];
      gyrset[1]+=gyr_lpf[1];
      gyrset[2]+=gyr_lpf[2];
      system_delay_ms(1);
  }
    gyrset[0] = gyrset[0] /1000;
    gyrset[1] = gyrset[1] /1000;
    gyrset[2] = gyrset[2] /1000;
    for(i=0;i<1000;i++){
    Attitude_get();
    }
    yaw=Yaw;
    pitch=Pitch;
    roll=Roll;
}
void zero()
{
    int i = 0;
    att_2.icm_ax = 0;
    att_2.icm_ay = 0;
    att_2.icm_az = 0;
    att_2.icm_gx = 0;
    att_2.icm_gy = 0;
    att_2.icm_gz = 0;

    att.icm_ax = 0;
    att.icm_ay = 0;
    att.icm_az = 0;
    att.icm_gx = 0;
    att.icm_gy = 0;
    att.icm_gz = 0;

    for(i=0;i<1000;i++)
    {
      imu660rb_get_acc();
      imu660rb_get_gyro();
      att.icm_ax =  ((float)imu660rb_acc_x * ZO) * alpha  + att.icm_ax * (1 - alpha);
    att.icm_ay =  ((float)imu660rb_acc_y * ZO) * alpha  + att.icm_ay * (1 - alpha);
    att.icm_az =  ((float)imu660rb_acc_z * ZO) * alpha  + att.icm_az * (1 - alpha);

        norm = sqrtf(att.icm_ax *att.icm_ax + att.icm_ay * att.icm_ay + att.icm_az * att.icm_az);
        att_2.icm_ax +=  (att.icm_ax/norm);
    att_2.icm_ay +=  (att.icm_ay/norm);
    att_2.icm_az +=  (att.icm_az/norm);

        att.icm_gx = ((float)imu660rb_gyro_x);
        att.icm_gy = ((float)imu660rb_gyro_y);
        att.icm_gz = ((float)imu660rb_gyro_z);

        att_2.icm_gx +=  att.icm_gx;
        att_2.icm_gy +=  att.icm_gy;
        att_2.icm_gz +=  att.icm_gz;
        system_delay_ms(5);
    }
  att_2.icm_ax = att_2.icm_ax * 0.002;
  att_2.icm_ay = att_2.icm_ay * 0.002;
  att_2.icm_az = att_2.icm_az * 0.002;

    att_2.icm_gx = att_2.icm_gx * 0.002;
  att_2.icm_gy = att_2.icm_gy * 0.002;
  att_2.icm_gz = att_2.icm_gz * 0.002;

    att.icm_ax = 0;
    att.icm_ay = 0;
    att.icm_az = 0;
    att.icm_gx = 0;
    att.icm_gy = 0;
    att.icm_gz = 0;
}
#define AcceRatio   4096.0f
#define GyroRatio   16.4f
#define ACC_FILTER_NUM 16
#define GYRO_FILTER_NUM 16
int ACC_X_BUF[ACC_FILTER_NUM], ACC_Y_BUF[ACC_FILTER_NUM], ACC_Z_BUF[ACC_FILTER_NUM];
int GYRO_X_BUF[GYRO_FILTER_NUM], GYRO_Y_BUF[GYRO_FILTER_NUM], GYRO_Z_BUF[GYRO_FILTER_NUM];


int buff_init=0;
int index=0;
void Data_Filter()
{

    long long temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0, temp5 = 0, temp6 = 0;
    ACC_X_BUF[0] = imu660rb_acc_x;
    ACC_Y_BUF[0] = imu660rb_acc_y;
    ACC_Z_BUF[0] = imu660rb_acc_z;
    GYRO_X_BUF[0] =imu660rb_gyro_x;
    GYRO_Y_BUF[0] =imu660rb_gyro_y;
    GYRO_Z_BUF[0] =imu660rb_gyro_z;
    if(buff_init==0){
        for(int i=0;i<ACC_FILTER_NUM-1;i++){
            imu660rb_get_acc();
            ACC_X_BUF[ACC_FILTER_NUM-1-i]=imu660rb_acc_x;
            ACC_Y_BUF[ACC_FILTER_NUM-1-i] = imu660rb_acc_y;
            ACC_Z_BUF[ACC_FILTER_NUM-1-i] = imu660rb_acc_z;
        }
        for(int i=0;i<GYRO_FILTER_NUM-1;i++){
            imu660rb_get_gyro();
            GYRO_X_BUF[GYRO_FILTER_NUM-1-i] =imu660rb_gyro_x;
            GYRO_Y_BUF[GYRO_FILTER_NUM-1-i] =imu660rb_gyro_y;
            GYRO_Z_BUF[GYRO_FILTER_NUM-1-i] =imu660rb_gyro_z;
        }
        buff_init=1;
    }



    for(int i=0;i<ACC_FILTER_NUM;i++)
    {
        temp1 += ACC_X_BUF[i];
        temp2 += ACC_Y_BUF[i];
        temp3 += ACC_Z_BUF[i];
    }
    for(int i=0;i<GYRO_FILTER_NUM;i++)
    {
        temp4 += GYRO_X_BUF[i];
        temp5 += GYRO_Y_BUF[i];
        temp6 += GYRO_Z_BUF[i];
    }

    acc_lpf[0] = (temp1+0.0) / ACC_FILTER_NUM;
    acc_lpf[1] = (temp2+0.0) / ACC_FILTER_NUM;
    acc_lpf[2] = (temp3+0.0) / ACC_FILTER_NUM;
    gyr_lpf[0] = (temp4+0.0) / GYRO_FILTER_NUM;
    gyr_lpf[1] = (temp5+0.0) / GYRO_FILTER_NUM;
    gyr_lpf[2] = (temp6+0.0) / GYRO_FILTER_NUM;

    for(int i = 0; i < ACC_FILTER_NUM - 1; i++)
    {
        ACC_X_BUF[ACC_FILTER_NUM-1-i] = ACC_X_BUF[ACC_FILTER_NUM-2-i];
        ACC_Y_BUF[ACC_FILTER_NUM-1-i] = ACC_Y_BUF[ACC_FILTER_NUM-2-i];
        ACC_Z_BUF[ACC_FILTER_NUM-1-i] = ACC_Z_BUF[ACC_FILTER_NUM-2-i];

    }
    for(int i = 0; i < GYRO_FILTER_NUM - 1; i++)
    {
        GYRO_X_BUF[GYRO_FILTER_NUM-1-i] = GYRO_X_BUF[GYRO_FILTER_NUM-2-i];
        GYRO_Y_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Y_BUF[GYRO_FILTER_NUM-2-i];
        GYRO_Z_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Z_BUF[GYRO_FILTER_NUM-2-i];
    }
}
void Data_Filter1(){


    acc_lpf[0] =  (acc_lpf[0] ) * (1-alpha) + (imu660rb_acc_x) * alpha;
    acc_lpf[1] =  (acc_lpf[1] ) * (1-alpha) + (imu660rb_acc_y) * alpha;
    acc_lpf[2] =  (acc_lpf[2] ) * (1-alpha) + (imu660rb_acc_z) * alpha;
    gyr_lpf[0] =  (gyr_lpf[0] ) * (1-alpha) + (imu660rb_gyro_x) * alpha;
    gyr_lpf[1] =  (gyr_lpf[1]) * (1-alpha) + (imu660rb_gyro_y) * alpha;
    gyr_lpf[2] =  (gyr_lpf[2] ) * (1-alpha) + (imu660rb_gyro_z) * alpha;
}
void Data_Filter2(){

}
void getimu_value()
{
    float x = 0,iorn = 0;
    int e = 0;
    imu660rb_get_acc();
    imu660rb_get_gyro();


    att.icm_ax =  ((float)imu660rb_acc_x * ZO) * alpha + att.icm_ax * (1 - alpha);
  att.icm_ay =  ((float)imu660rb_acc_y * ZO) * alpha + att.icm_ay * (1 - alpha);
  att.icm_az =  ((float)imu660rb_acc_z * ZO) * alpha + att.icm_az * (1 - alpha);
    ax = att.icm_ax;
    ay = att.icm_ay;
    az = att.icm_az;

    att.icm_gx = (((float)imu660rb_gyro_x)- att_2.icm_gx ) * XO;
  att.icm_gy = (((float)imu660rb_gyro_y)- att_2.icm_gy ) * XO;

    for(e = 0; e < 4;e++)
    {
        yaw_speed[e] = yaw_speed[e+1];
        iorn += yaw_speed[e];
    }
    yaw_speed[e] = ((float)imu660rb_gyro_z - att_2.icm_gz)/16.4f;
    angle_speed =angle_speed * 0.65 + 0.35 *((iorn + yaw_speed[e])*0.2);

  att.icm_gz = (((float)imu660rb_gyro_z)- att_2.icm_gz ) * XO;
    gx = att.icm_gx;
    gy = att.icm_gy;
    gz = att.icm_gz;
    attitude_solution();

}
float q00=1.0,q11=0,q22=0,q33=0,q44=0;
float eulerAngle_yaw=0;
void attitude_solution()
{
    float q0q0 = q00*q00, q0q1 = q00*q11, q0q2 = q00*q22, q0q3 = q00*q33, q1q1 = q11*q11, q1q2 = q11*q22;
    float q1q3 = q11*q33, q2q2 = q22*q22, q2q3 = q22*q33, q3q3 = q33*q33;


    norm = 1/sqrtf(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    posture_x = 2*(q1q3 - q0q2);
    posture_y = 2*(q0q1 + q2q3);
    posture_z = q0q0 - q1q1 - q2q2 + q3q3;


    error_x = (ay*posture_z - az*posture_y) ;
    error_y = (az*posture_x - ax*posture_z) ;
    error_z = (ax*posture_y - ay*posture_x) ;

    xErrorInt = xErrorInt + error_x * halfT * 0.5;
    yErrorInt = yErrorInt + error_y * halfT * 0.5;
    zErrorInt = zErrorInt + error_z * halfT * 0.5;


    gx = gx + Kp2*error_x + xErrorInt * Ki2;
    gy = gy + Kp2*error_y + yErrorInt * Ki2;
    gz = gz + Kp2*error_z + zErrorInt * Ki2;


    q0temp=q00;
    q1temp=q11;
    q2temp=q22;
    q3temp=q33;

        q00 = q00 + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
    q11 = q11 + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
    q22 = q22 + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
    q33 = q33 + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;



    norm = 1/sqrtf(q00*q00 + q11*q11 + q22*q22 + q33*q33);

    q00 = q00 * norm;
    q11 = q11 * norm;
    q22 = q22 * norm;
    q33 = q33 * norm;

    q0q1 = q00*q11; q0q2 = q00*q22; q0q3 = q00*q33; q1q1 = q11*q11; q1q2 = q11*q22;
    q1q3 = q11*q33; q2q2 = q22*q22; q2q3 = q22*q33; q3q3 = q33*q33;


     Yaw  = 57.3*atan2f(2.f * (q0q3 + q1q2),1 - 2.f * (q2q2 + q3q3));
     eulerAngle_yaw=-Yaw;
     if(Yaw<0){
              yaw1=360+(int)Yaw;
              }else{
                  yaw1=(int)Yaw;
              }

}
