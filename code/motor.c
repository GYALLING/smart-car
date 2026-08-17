#include "motor.h"

#define PWMC                 (ATOM0_CH3_P21_5)
#define DIRC                 (P21_4)
/*
 * 初始化电机
 */
void MotorInit (void)
{
    small_driver_uart_init();       // 初始化驱动通讯功能

    gpio_init(DIRC, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(PWMC, 17000, 0);
}
/*
 * 设置占空比PWM
 */
void MotorCtrl3W(int motora,int motorb,int motorc)
{
    small_driver_set_duty(-motora,motorb);
    if(motorc >0)
    {
        gpio_set_level(DIRC, GPIO_LOW);
        pwm_set_duty (PWMC,motorc);
    }
    else
    {
        gpio_set_level(DIRC, GPIO_HIGH);
        pwm_set_duty (PWMC,-motorc);
    }
}
/*
 * 编码器初始化
 */
void encoder_init ()
{
    encoder_dir_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
    // 使用T4定时器   P02_8引脚进行计数    计数方向使用P00_9引脚  TIM4是对行进轮 PWM3
}
/*
 * 获取行进轮速度
 */
double v=0;
void encoder_speed_get()
{
    //PWMC对应编码器读取
    speedc = encoder_get_count(TIM4_ENCODER);                                         //get_count
    encoder_clear_count (TIM4_ENCODER);
    v=speedc*0.0000544/0.01;
}


