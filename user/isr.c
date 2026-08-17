/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
********************************************************************************************************************/
#include "isr_config.h"
#include "isr.h"
extern vb;
// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。

// **************************** PIT中断函数 ****************************

int16 speedc=0;//行进轮速度
int k;
int count=0,count1=0,bcount;
float distance=0;
int16 velocity=0;
int16 velocity_goal=70;                //目标行进速度50
int16 velocity_goal1=55;                //目标行进速度50
extern double target_yaw_goal ;
short unbaidian_time=0;                 //没有检测到白点计时器
double K1=1;  //6-    1   3  3          2 1//压弯系数2.5,2.5//起始点 2.5
double K2=8;//10+    7 /8 8    16 18     8  12        10
double K3=0.01;//100+  定义为新的angle_err限幅系数 0.02
double K4=0;
double angle_err_kp=0,qianhou_err_kp=0; //动态零点控制参数
//double angle_err,   Roll_zero_value=-0.7;    //左右动态零点偏差；左右机械零点
//double qianhou_err, Pitch_zero_value=-3.0; //前后动态零点偏差；前后机械零点
double angle_err=0,angle_err1=0,angle_err2=0,Pitch_zero_value=1.2;    //左右动态零点偏差；左右机械零点12.2v 二队1.4 一队1.8
double qianhou_err=0, Roll_zero_value=2.1; //前后动态零点偏差；前后机械零点二队2.1 一对-0.8
short guiling_time=0;                   //转向角归零定时器；
bool guiling_flag=true;                 //是否归零标志位；
bool fangxiang=false;
int Motor_a,Motor_b,Motor_c;            //动量轮PWM，行进轮PWM
short count6ms,count10ms,count20ms,vc,jiaoduc;
short count3000ms=0;
IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

//    imu660ra_get_acc();
//    imu660ra_get_gyro();
    imu660rb_get_acc();
    imu660rb_get_gyro();
    Data_Filter();
    //PWM_A=jiaojiasudu_A(Gyro[0],PWM_A_jiaodu);                  //动量轮角速度环PID位置式
//    PWM_A=jiaojiasudu_A_zengliang(Gyro[0],PWM_A_jiaodu);      //动量轮角速度环PID增量式
//    //PWM_C=jiaojiasudu_C(Gyro[1],PWM_C_jiaodu);                  //行进轮角速度环PID位置式
//    PWM_C=jiaojiasudu_C_zengliang( Gyro[1],  PWM_C_jiaodu);   //行进轮角速度环PID增量式
//    //PWM_C=balance_UP_C(Pitch,Pitch_zero,Gyro[1]);             //行进轮并联PID控制
//    PWM_yaw=jiaojiasudu_yaw(Gyro[2],PWM_yaw_jiaodu);            //转向PID

    PWM_A=jiaojiasudu_A_zengliang(Gyro[1],PWM_A_jiaodu);
    PWM_C=jiaojiasudu_C_zengliang(-Gyro[0],  PWM_C_jiaodu);
    PWM_yaw=jiaojiasudu_yaw(Gyro[2],PWM_yaw_jiaodu);
    Motor_a = (int)(PWM_A - PWM_yaw);
    Motor_b = (int)(PWM_A +PWM_yaw);
//    if(PWM_C>0)
//        PWM_C+=900;
//    else
//        if(PWM_C<0)
//            PWM_C-=900;
//        else
//            PWM_C=0;//一队加
    Motor_c=(int)(PWM_C);
    Motor_a=constrain_int(Motor_a,-10000,10000);  //限幅
    Motor_b=constrain_int(Motor_b,-10000,10000);
    Motor_c=constrain_int(Motor_c,-5000,5000);
    if(Roll>35||Roll<-35||Pitch>30||Pitch<-30)                       //车倒后点击停转
        MotorCtrl3W(0,0,0);
    else
        MotorCtrl3W(Motor_a,Motor_b,Motor_c);

    if(count10ms>=5)
    {
        count10ms=0;
        Attitude_get();
        getimu_value();
        //angle_err=angle_err_kp/100000*1*fabs(speedc);        //左右期望零点偏差计算
        //qianhou_err=qianhou_err_kp/100000*1*fabs(speedc);    //前后期望零点偏差计算
//        Roll_zero=Roll_zero_value+angle_err;                 //左右动态零点
//        Pitch_zero=Pitch_zero_value+qianhou_err;             //前后动态零点
        Roll_zero=Roll_zero_value+angle_err2;                 //前后动态零点
        Pitch_zero=Pitch_zero_value+angle_err;             //左右动态零点
//        Roll_zero=constrain_float(Roll_zero,-60,60);         //限幅
//        Pitch_zero=constrain_float(Pitch_zero,-20,20);       //限幅
//        PWM_A_jiaodu=jiaodu_A(Roll,Roll_zero+PWMa_accel);    //动量轮角度环PID
//        PWM_C_jiaodu=jiaodu_C(Pitch,Pitch_zero+0+PWMc_accel);//行进轮角度环PID
        PWM_A_jiaodu=jiaodu_A(Pitch,Pitch_zero+PWMa_accel);    //动量轮角度环PID
        PWM_C_jiaodu=jiaodu_C(-Roll,Roll_zero+PWMc_accel);//行进轮角度环PID
//      bai_num=0; //测试位
        // 假设每20ms执行一次（根据你的PIT中断频率调整）

         if (bai_num > 0 )                                        //当没有识别到信标时转弯
        {

            count = 0;
            unbaidian_time = 0;
            get_target_yaw(baidian);
            distance=get_distance(baidian);

//            if (target_yaw < target_yaw_goal)
//                target_yaw = target_yaw + 0.25;
//            else if(target_yaw > target_yaw_goal)
//                target_yaw = target_yaw - 0.25;



            if(target_yaw_goal<0)
            {
                K1=3;
                K2=8;
            }
            else if(target_yaw_goal>0)
            {
                K1=3;
                K2=8;
            }

            if (Pitch <6 && Pitch > -6)
            {
                if (target_yaw < target_yaw_goal)
                    target_yaw = target_yaw + 0.25;
                else if(target_yaw > target_yaw_goal)
                    target_yaw = target_yaw - 0.25;
            }
            else if(Pitch >6 || Pitch < -6)
            {
                if (target_yaw < target_yaw_goal)
                    target_yaw = target_yaw + 0.15;
                else if(target_yaw > target_yaw_goal)
                    target_yaw = target_yaw - 0.15;
            }

//            if(distance<70)
//            {
//                if(velocity_goal>50)
//                velocity_goal = velocity_goal -0.5;
//             }
//            else{
            if (fabs(target_yaw-target_yaw_goal)<1&&fabs(target_yaw)<10)//Pitch < 1 && Pitch > -1
            {
                K3=0;//0.01
               if (velocity_goal < 120)
                velocity_goal = velocity_goal + 1;
            }
            else
            {
                K3=0;
            }

            if (fabs(target_yaw-target_yaw_goal)<1&&fabs(target_yaw)<20&&fabs(target_yaw)>10)//Pitch < 1 && Pitch > -1
            {
                if (velocity_goal < 100)
                    velocity_goal = velocity_goal + 1;
                if (velocity_goal > 100)
                   velocity_goal = velocity_goal - 1;
            }

            if (fabs(target_yaw)>20&&fabs(target_yaw_goal)>20)//Pitch < 1 && Pitch > -1
            {
                if (velocity_goal < 90)
                    velocity_goal = velocity_goal + 1;
                if (velocity_goal > 90)
                    velocity_goal = velocity_goal - 1;
            }

            if(distance<70)
            {
                K3=0;
              if(velocity_goal>80)//70
               velocity_goal = velocity_goal -1;
            }
        }
        else
        {
            if(unbaidian_time >= 150 && unbaidian_time <550)//180
            {
                if(fangxiang==false)//zhuo
                {
                    K2=5;
                }
                else if(fangxiang==true)
                {
                    K2=7;
                }

                if(target_yaw>0&&velocity_goal>=55)velocity_goal=velocity_goal-0.5;
                if(target_yaw>0&&velocity_goal<=55)velocity_goal=velocity_goal+0.5;
                if(target_yaw<0&&velocity_goal>=55)velocity_goal=velocity_goal-0.5;
                if(target_yaw<0&&velocity_goal<=55)velocity_goal=velocity_goal+0.5;

                if(target_yaw>=0&&target_yaw<40&&fangxiang==false)target_yaw=target_yaw+0.2;//向左
                else if(target_yaw<=0&&target_yaw>-40&&fangxiang==true)target_yaw=target_yaw-0.2;//向右
                unbaidian_time++;
            }
            else if(unbaidian_time >=550 && unbaidian_time <=850)// 650 1050
                        {
            //                count=0;
//                   K2=0;


                            if (target_yaw > 0)
                            {
                                target_yaw = target_yaw - 0.2;
                                if(target_yaw > 0 && target_yaw <3)
                                {
                                    target_yaw = 0;
                                }

                            }
                            else if (target_yaw < 0)
                            {
                                target_yaw = target_yaw + 0.2;
                                 if(target_yaw < 0 && target_yaw >-3)
                                {
                                    target_yaw = 0;
                                }
                            }

                            if (Pitch < 3 && Pitch > -3&&(target_yaw<5&&target_yaw>-5))
                            {
                                if (velocity_goal > 60)
                                    velocity_goal = velocity_goal - 1;
                            }

                            //target_yaw = 0;
                            if(fangxiang==true)
                                fangxiang=false;
                            else if(fangxiang==false)
                                fangxiang=true;

                            unbaidian_time++;

                        }
             else if (unbaidian_time >= 850 && unbaidian_time <1250)//270
            {
//                   count = count - 0.5;//2.0版
//                   target_yaw = -30;
//                   target_yaw =vb*(target_yaw + count);
//                    if (fabs(target_yaw) >= 40){
//                        target_yaw = -30*vb;
//                    }

                 if(fangxiang==false)
                 {
                     K2=5;
                 }
                 else if(fangxiang==true)
                 {
                     K2=7;
                 }

                    if(target_yaw>=0&&target_yaw<40&&fangxiang==false)target_yaw=target_yaw+0.2;//向左
                    else if(target_yaw<=0&&target_yaw>-40&&fangxiang==true)target_yaw=target_yaw-0.2;//向右

                    if(target_yaw>0&&velocity_goal>=55)velocity_goal=velocity_goal-0.5;
                    if(target_yaw>0&&velocity_goal<=55)velocity_goal=velocity_goal+0.5;
                    if(target_yaw<0&&velocity_goal>=55)velocity_goal=velocity_goal-0.5;
                    if(target_yaw<0&&velocity_goal<=55)velocity_goal=velocity_goal+0.5;

                    unbaidian_time++;
            }

            else if(unbaidian_time >1250)//1050
            {
                unbaidian_time=552;
//                target_yaw = 0;
//                if(fangxiang==true)
//                    fangxiang=false;
//                else if(fangxiang==false)
//                    fangxiang=true;


            }
            else
            {
//                if (target_yaw > 0)     //               target_yaw = 0;//100 de直行
//                    target_yaw = target_yaw - 0.05;
//                else if (target_yaw < 0)
//                    target_yaw = target_yaw + 0.05;

                if(velocity_goal>70)
                {
                    velocity_goal-=1;
//                    qianhou_err=-2;
                }

                target_yaw_goal=0;
                if (target_yaw < target_yaw_goal)
                {
                    target_yaw = target_yaw + 0.2;
                    if(target_yaw<0&&target_yaw>-3)
                        {
                        target_yaw=-1;
                        }
                }
                else if (target_yaw > target_yaw_goal)
                {
                    target_yaw = target_yaw - 0.2;
                    if(target_yaw<3&&target_yaw>0){
                           target_yaw=1;}
                }

                if(velocity_goal<=70&&target_yaw<3&&target_yaw>-3)
                {
                        unbaidian_time=151;
                }


                    if(target_yaw>0)
                    {
                        fangxiang=false;
                    }
                    else if(target_yaw<0)
                    {
                        fangxiang=true;
                    }
                    K3=0;
                unbaidian_time++;
            }

//




//                    count = count - 0.5;//1.0版//右转
//                    target_yaw = -30;
//                    target_yaw =target_yaw + count;
//                     if (fabs(target_yaw) >= 40){
//                         target_yaw = -30;
//                     }
//                                    count = count + 0.5;//左转
//                                    target_yaw = 30;
//                                    target_yaw = target_yaw + count;
//                                    if (target_yaw >= 40){
//                                        target_yaw = 30;
//                                    }

            /**
             *  分级搜索策略实现
             */
/*            static void search_strategy(void)
            {
                if (unbaidian_time < 100) {
                    // 阶段1：短暂等待（直行100ms）
                    target_yaw = 0;
                }
                else if (unbaidian_time>=100 && unbaidian_time < 3140) {
                    // 阶段2：正弦摆动（±30°，持续200ms）
                    target_yaw = 40 * sin(unbaidian_time * 0.001);
                }
                else {
                    // 阶段3：螺旋扩展（50°~110°周期性递增）
            //        target_yaw = 30 + (unbaidian_time % 60);
            //        target_yaw =constrain_float(target_yaw,-40,40);
            //        target_yaw = 40;
                    if (target_yaw > 0)
                        target_yaw = target_yaw - 0.05;
                    else if (target_yaw < 0)
                        target_yaw = target_yaw + 0.05;

                    if (Pitch < 1 && Pitch > -1&&(target_yaw<10&&target_yaw>-10))
                    {
                        if (velocity_goal < 70)
                            velocity_goal = velocity_goal + 1;
                    }
                }
            }*/

//                 bcount=1;
//                if(bcount==1)
//                 {
//                    bcount = 0;
//                    count1++;
//                }
//                if (count1 % 2 != 0)
//                {
//                if(velocity>velocity_goal1)
//                {velocity-=0.5;}//在压弯时如果没有找到灯速度-- 方便压弯
//                else if(velocity<=velocity_goal1)
//                {velocity=velocity_goal1;}

//                target_yaw = 30;//
//                if (unbaidian_time < 200)
//                {
//                    target_yaw = target_yaw + (unbaidian_time - 100) * 0.1; // 逐渐增加转向角度
//                }
//                else if(unbaidian_time > 200 &&unbaidian_time < 300)
//                {
//                    target_yaw =target_yaw - (unbaidian_time - 200) * 0.1; // 逐渐减少转向角度
//                }
//                else
//                {
//                    target_yaw = 30;
//                }

//                    count = count +0.5;
//                    target_yaw = 30;
//                     if (target_yaw <=40)
//                      target_yaw = target_yaw + count;


//                }
//                else
//                {
////                    count = count + 0.5;
//                    target_yaw = 40;
////                    if (target_yaw >= 0)
////                        target_yaw = target_yaw - count;
//
//                }

//            else if(unbaidian_time >= 120)
//            {
//                       count = count + 0.5;
//                       target_yaw = 40;
//                       if (target_yaw >= 0)
//                       {
//                           target_yaw = target_yaw - count;
//                       }
//            }


//            else
//            {
//                if (target_yaw > 0)     //               target_yaw = 0;//100 de直行
//                    target_yaw = target_yaw - 0.05;
//                else if (target_yaw < 0)
//                    target_yaw = target_yaw + 0.05;
//
//                unbaidian_time++;
//            }

        }

//       target_yaw =0;//测试位
       smoothUpdate(&target_yaw,&target_yaw_err,0.1);//0.05
       PWM_yaw_jiaodu=jiaodu_yaw(target_yaw,PWMyaw_accel);//target_yaw应取反 正左负右
//     target_yaw_err=target_yaw;
//       angle_err2=-target_yaw*K3;
//       if(angle_err2>8)//target_yaw*speedc*K4
//       {
//           angle_err2=8;
//       }
//       else if(angle_err2<-8)
//       {
//           angle_err2=-8;
//       }

//       qianhou_err=-(K3*fabs(target_yaw))/100*(speedc/100)*(speedc/100)/(2*0.35);//100+
//
//        if(qianhou_err<-10)
//        {
//            qianhou_err=-10;
//        }
//        Roll_zero=qianhou_err+Roll_zero_value;
       angle_err2=fabs(speedc*K3)-fabs(target_yaw)*K3*100/40;
       if(angle_err2>1)//target_yaw*speedc*K4
       {
           angle_err2=1;
       }
       else if(angle_err2<0)
       {
           angle_err2=0;
       }
       Roll_zero=angle_err2+Roll_zero_value;




       angle_err=-(K2*(target_yaw)/100+K1*speedc/100*speedc/100*(target_yaw)/100);
//       angle_err = constrain_double(angle_err, -fabs(K3) * fabs(speedc) * fabs(target_yaw)/10000, fabs(K3) * fabs(speedc) * fabs(target_yaw)/10000);
        if(angle_err>12.5)
        {
            angle_err=12.5;
        }
        if(angle_err<-14)//左压弯
        {
            angle_err=-14;
        }

       Pitch_zero=Pitch_zero_value+angle_err;
    }
    if(count20ms>=10)
    {
        count20ms=0;
        encoder_speed_get();
        PWMa_accel=Velocity_Control_A(-(speeda-speedb)/2);
        if(velocity>velocity_goal)
        {velocity-=1;}//在压弯时如果没有找到灯速度-- 方便压弯
        else if(velocity<velocity_goal)
        {velocity+=2;}
        PWMc_accel=Velocity_Control_C(speedc+velocity);
        //Velocity(speedc);
        PWMyaw_accel=Velocity_Control_yaw(speeda+speedb);
    }

    count10ms++;
    count20ms++;
    count3000ms++;

}
//                if(bai_num>0)                                        //当没有识别到信标时转弯
//                        {
//
//                            unbaidian_time=0;
//                            get_target_yaw(baidian);
//                            if(target_yaw>=40)
//                            {
//                                target_yaw=50;
//                            }
//                            else if(target_yaw<=-40)
//                            {
//                                target_yaw=-50;
//                            }
//                            if (target_yaw_err < target_yaw)
//                            {
//                                    target_yaw_err += 5;
//                                    if (target_yaw_err > target_yaw)  // 防止超调
//                                        target_yaw_err = target_yaw;
//                            }
//                            else if (target_yaw_err > target_yaw)
//                                {
//                                    target_yaw_err -= 5;
//                                    if (target_yaw_err < target_yaw)  // 防止超调
//                                        target_yaw_err = target_yaw;
//                                }
//                        }
//                        else
//                        {
//                            if(unbaidian_time>=100)
//                            {
//                                target_yaw=-40;
//                                target_yaw_err = target_yaw;
//                            }
//                            else
//                            {
//                                target_yaw=0;
//                                target_yaw_err = target_yaw;
//                                unbaidian_time++;
//                            }
//                        }
////                target_yaw=50;
//        if(bai_num>0)                                        //当没有识别到信标时转弯
//        {
//            unbaidian_time=0;
//            get_target_yaw(baidian);
//            if(target_yaw>=40)
//            {
//                target_yaw_err=50;
//
//            }
//            else if(target_yaw<=-40)
//            {
//                target_yaw_err=-50;
//
//            }
//              if (target_yaw_err > target_yaw)
//            {
//               target_yaw_err -= 0.2;
//               if (target_yaw_err < target_yaw)
//                 target_yaw_err = target_yaw;  // 避免超调
//            }
//                else if (target_yaw_err < target_yaw)
//              {
//                    target_yaw_err += 0.2;
//                    if (target_yaw_err > target_yaw)
//                      target_yaw_err = target_yaw;  // 避免超调
//              }
//        }
//        else
//        {
//            if(unbaidian_time>=100)
//            {
//                target_yaw=-40;
//                target_yaw_err=target_yaw;
//
//            }
//            else
//            {
//                target_yaw=0;
//                target_yaw_err=target_yaw;
//                unbaidian_time++;
//            }
//        }

IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);




}

IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);




}

IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_VECTAB_NUM, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);





}
// **************************** PIT中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, EXTI_CH0_CH4_INT_VECTAB_NUM, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);



    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);




    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, EXTI_CH1_CH5_INT_VECTAB_NUM, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF 模块 INT 更新中断

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);



    }
}

// 由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, EXTI_CH2_CH6_INT_VECTAB_NUM, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, EXTI_CH3_CH7_INT_VECTAB_NUM, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // 摄像头触发采集统一回调函数
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);




    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, DMA_INT_VECTAB_NUM, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler();                           // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, UART0_INT_VECTAB_NUM, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
IFX_INTERRUPT(uart0_rx_isr, UART0_INT_VECTAB_NUM, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

#if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                  // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
}


// 串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, UART1_INT_VECTAB_NUM, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套




}
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_uart_handler();                          // 摄像头参数配置统一回调函数
}

// 串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_VECTAB_NUM, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart2_rx_isr, UART2_INT_VECTAB_NUM, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    wireless_module_uart_handler();                 // 无线模块统一回调函数



}
// 串口3默认连接到GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, UART3_INT_VECTAB_NUM, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart3_rx_isr, UART3_INT_VECTAB_NUM, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    //gnss_uart_callback();                           // GNSS串口回调函数
    uart_control_callback();


}


IFX_INTERRUPT(uart4_tx_isr, UART4_INT_VECTAB_NUM, UART4_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart4_rx_isr, UART4_INT_VECTAB_NUM, UART4_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart5_tx_isr, UART5_INT_VECTAB_NUM, UART5_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart5_rx_isr, UART5_INT_VECTAB_NUM, UART5_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart6_tx_isr, UART6_INT_VECTAB_NUM, UART6_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart6_rx_isr, UART6_INT_VECTAB_NUM, UART6_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart8_tx_isr, UART8_INT_VECTAB_NUM, UART8_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart8_rx_isr, UART8_INT_VECTAB_NUM, UART8_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart9_tx_isr, UART9_INT_VECTAB_NUM, UART9_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart9_rx_isr, UART9_INT_VECTAB_NUM, UART9_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart10_tx_isr, UART10_INT_VECTAB_NUM, UART10_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart10_rx_isr, UART10_INT_VECTAB_NUM, UART10_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart11_tx_isr, UART11_INT_VECTAB_NUM, UART11_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart11_rx_isr, UART11_INT_VECTAB_NUM, UART11_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, UART0_INT_VECTAB_NUM, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, UART1_INT_VECTAB_NUM, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, UART2_INT_VECTAB_NUM, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, UART3_INT_VECTAB_NUM, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
IFX_INTERRUPT(uart4_er_isr, UART4_INT_VECTAB_NUM, UART4_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart4_handle);
}
IFX_INTERRUPT(uart5_er_isr, UART5_INT_VECTAB_NUM, UART5_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart5_handle);
}
IFX_INTERRUPT(uart6_er_isr, UART6_INT_VECTAB_NUM, UART6_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart6_handle);
}
IFX_INTERRUPT(uart8_er_isr, UART8_INT_VECTAB_NUM, UART8_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart8_handle);
}
IFX_INTERRUPT(uart9_er_isr, UART9_INT_VECTAB_NUM, UART9_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart9_handle);
}
IFX_INTERRUPT(uart10_er_isr, UART10_INT_VECTAB_NUM, UART10_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart10_handle);
}
IFX_INTERRUPT(uart11_er_isr, UART11_INT_VECTAB_NUM, UART11_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart11_handle);
}
// **************************** 串口中断函数 ****************************
