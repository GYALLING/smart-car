#ifndef USER_IMAGE_H_
#define USER_IMAGE_H_
#include"zf_common_headfile.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <stdint.h>
#define BLACK 0
#define WHITE 255
#define MAX_LABELS 200
#define BLOCK_SIZE 10
#define che_x (85)   //车在图像中的虚拟位置x坐标
#define che_y (119)  //车在图像中的虚拟位置y坐标
// 连通域结构体（保存区域特征）
typedef struct
{
    uint16_t x;         // 左上角x坐标
    uint16_t y;         // 左上角y坐标
    uint8_t x_min;
    uint8_t x_max;
    uint8_t y_max;
    uint8_t y_min;
    uint8_t width;     // 区域宽度
    uint8_t height;    // 区域高度
    uint16_t area;      // 像素面积
    uint16_t centroid_x;    // 新增：横向质心坐标 [[3]][[4]]
    uint16_t centroid_y;// 纵向重心坐标
    uint16_t max_x;      // 新增最大X坐标
    uint16_t max_y;     // 新增最大Y坐标
    uint16_t now_x;
    uint16_t now_y;
    uint16_t max_gray;     // 区域最大灰度值
    uint16_t max_gray_x;   // 最大灰度值坐标X
    uint16_t max_gray_y;   // 最大灰度值坐标Y
    float distance;
    float angle;


} Region;
#define MIN_AREA 1
#define NEAR_THRESHOLD 2
typedef struct
{
    int x, y;
    float dist;
} BlockResult;
typedef struct whiteaxis
{
    short x;
    short y;
}whiteaxis;
extern double target_yaw;
extern whiteaxis baidian;
extern int bai_num;
extern int white_pixel_count;

extern double target_yaw_err;
extern Region  regions[200];;
void shibie3(uint8 image[][MT9V03X_W],int HEIGHT,int WIDTH);
void shibie4(uint8 image[][MT9V03X_W],int HEIGHT, int WIDTH);
void get_target_yaw(whiteaxis baidian);
void Bin_Image_Filter (uint8 image[][MT9V03X_W],int HEIGHT, int WIDTH);
uint16 otsuThreshold(uint8 *image, uint16 width, uint16 height);
uint8_t zuijin_xiabiao (Region regions[],uint8_t region_count);
float get_distance(whiteaxis bai);
bool if_xinbiao(Region region);
void binarization(uint8 *image,uint16 width, uint16 height,uint16 threshold);
uint8_t analyze_with_binarize(uint8_t threshold, Region *regions);
double get_angle(whiteaxis bai);
#endif
