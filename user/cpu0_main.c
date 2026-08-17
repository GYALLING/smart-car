
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
* 文件名称          cpu0_main
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
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
uint8 image[MT9V03X_H][MT9V03X_W];
extern int count;
 extern area;
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    ips200_init(IPS200_TYPE_SPI);   //屏幕初始化
    mt9v03x_init();                 //摄像头初始化
    //imu660ra_init ();             //6轴陀螺仪初始化
    imu660rb_init();
    //imu963ra_init();                //9轴陀螺仪初始化
    MotorInit();                    //电机初始化
    Zero_Attitude();
    encoder_init();                 //行进轮编码器初始化

    pit_ms_init(CCU60_CH0, 1);      //中断初始化
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        if(mt9v03x_finish_flag)
        {
//            for(int h=0;h<MT9V03X_H;h++)
//            {
//                for(int w=0;w<MT9V03X_W;w++)
//                {
//                    image[h][w]=mt9v03x_image[h][w];       //拷贝图像
//                }
//            }
//            binarization(image,MT9V03X_W,MT9V03X_H,120);    //图像二值化
            //Bin_Image_Filter (image, MT9V03X_H, MT9V03X_W);
            //shibie(image,MT9V03X_H,MT9V03X_W);
            //shibie2(image,MT9V03X_H,MT9V03X_W);            //识别白点
            //shibie3(image, MT9V03X_H, MT9V03X_W);
         //   uint16 threshold= otsuThreshold(mt9v03x_image, MT9V03X_W, MT9V03X_H);
          //  binarization(mt9v03x_image,MT9V03X_W,MT9V03X_H,threshold);    //图像二值化

            ips200_show_gray_image(0,0,mt9v03x_image,MT9V03X_W,MT9V03X_H,188,120,85);//显示图像//85-95
            bai_num = 0;
            uint8_t region_count = analyze_with_binarize(85, regions);
            if (region_count > 0)
            {
                uint8_t xiabiao = zuijin_xiabiao(regions, region_count);
                if (xiabiao != 200)
                {

//                    baidian.x = regions[xiabiao].centroid_x;//-regions[xiabiao].width
                    if(regions[xiabiao].centroid_x>che_x){//左侧
                        baidian.x = regions[xiabiao].centroid_x-regions[xiabiao].width;//-regions[xiabiao].width

                    }
                    if(regions[xiabiao].centroid_x<che_x){
                        baidian.x = regions[xiabiao].centroid_x+regions[xiabiao].width;//-regions[xiabiao].width
                    }

                    baidian.y = regions[xiabiao].centroid_y;
                    bai_num++;
                    //                ips200_draw_line(regions[0].max_gray_x-20,regions[0].max_gray_y,regions[0].max_gray_x,regions[0].max_gray_y,RGB565_RED);
                    //                ips200_draw_line(regions[0].max_gray_x+20,regions[0].max_gray_y,regions[0].max_gray_x,regions[0].max_gray_y,RGB565_RED);
                    //                ips200_draw_line(regions[0].max_gray_x,regions[0].max_gray_y+20,regions[0].max_gray_x,regions[0].max_gray_y,RGB565_RED);
                    //                ips200_draw_line(regions[0].max_gray_x,regions[0].max_gray_y-20,regions[0].max_gray_x,regions[0].max_gray_y,RGB565_RED);
                    ips200_draw_line(che_x, che_y, baidian.x, baidian.y,RGB565_RED);
//                    ips200_show_float(0, 170, baidian.x, 3, 4);
//                    ips200_show_float(0, 190, baidian.y, 3, 4);
                    //                ips200_show_int(0,170,regions[xiabiao].max_gray_x,3);
                    //                ips200_show_int(30, 170, regions[xiabiao].max_gray_y, 3);
                    //                ips200_show_int(0, 190, regions[xiabiao].max_gray, 3);
                }
            }
            mt9v03x_finish_flag = 0;                       //重置标志位
        }




        ips200_show_float(0, 170, distance, 5, 6);
        ips200_show_float(0, 190, -target_yaw, 5, 6);
//        ips200_show_float(0,270, -target_yaw, 5, 6);
//        ips200_show_float(0, 190, unbaidian_time, 5, 6);

//        ips200_show_float(0, 150, speedc, 5, 6);
//        ips200_show_float(0, 170, speedb, 5, 6);
//        ips200_show_float(0, 210, v, 5, 6);
//            ips200_show_float(0, 190,  distance, 5, 6);
            ips200_show_float(0, 210, Pitch, 5, 6);
            ips200_show_float(0, 230, Roll, 5, 6);
//        ips200_show_float(0, 230, PWM_yaw, 5, 6);
//        ips200_show_float(0, 250, Gyro[2], 5, 6);
        ips200_show_float(0, 250,  speedc, 5, 6);
        //Roll和Pitch调换，并且Roll要取反
        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************




