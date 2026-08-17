#include"image.h"

double target_yaw_err;

whiteaxis baidian;  //白点信息x，y坐标
int bai_num = 0;    //识别到白点数量

/*
 * image[][]图像数组
 * im_h图像长
 * im_h图像宽
 * 作用：从车坐标开始按照先左右后向前的方式找白点
 */
void shibie(uint8 image[][MT9V03X_W], int im_h, int im_w)
{
    bai_num = 0;
    for (int i = che_x; i < im_h&&bai_num==0; i++)
    {
        for (int j = 0; j < im_w-che_y&&bai_num==0; j++)
        {
            if (image[i][che_y-j] == WHITE)
            {
                baidian.x = i;
                baidian.y = che_y-j;
                bai_num++;
            }
            else
                if(image[i][che_y+j] == WHITE)
                {
                    baidian.x=i;
                    baidian.y=che_y+j;
                    bai_num++;
                }
        }
    }
}
/*
 * image[][]图像数组
 * im_h图像长
 * im_w图像宽
 * 作用：从车坐标开始以矩形向外扩散的方式找白点；
 */
void shibie2(uint8 image[][MT9V03X_W],int im_h,int im_w)
{
    bai_num=0;
    for(int r=1;r<im_h-1;r++)
    {
        for(int i=0;i<=r&&che_y-i>=0&&che_y+i<im_w&&bai_num==0;i++)
        {
            if(image[che_x+r][che_y-i]==WHITE)
            {
                baidian.x = che_x+r;
                baidian.y = che_y-i;
                bai_num++;
            }
            else
                if(image[che_x+r][che_y+i]==WHITE)
                {
                    baidian.x = che_x+r;
                    baidian.y = che_y+i;
                    bai_num++;
                }
        }
        for(int j=0;j<=r&&bai_num==0;j++)
        {
            int y,y2;
            if(che_y-r<0)y=0;else y=che_y-r;
            if(che_y+r>=im_w)y2=im_w-1;else y2=che_y+r;
            if(image[che_x+j][y]==WHITE)
            {
                baidian.x = che_x+j;
                baidian.y = y;
                bai_num++;
            }
            else
                if(image[che_x+j][y2])
                {
                    baidian.x = che_x+j;
                    baidian.y = y2;
                    bai_num++;
                }
        }
    }
//    if(bai_num>0&&baidian.x<=30)
//    {
//        bai_num=0;
//    }
}
/*
 * image[][]图像数组
 * im_h图像长
 * im_w图像宽
 * 作用：从车坐标开始以半圆向外扩散的方式找白点；
 */
void shibie3(uint8 image[][MT9V03X_W],int HEIGHT,int WIDTH)
{
    bai_num=0;
//    int max_radius = (int)sqrt((WIDTH/2)*(WIDTH/2) + (HEIGHT/2)*(HEIGHT/2)) + 1;//最大半径
    int max_radius = (int)sqrt(107.0*107.0+120.0*120.0) + 1;//最大半径
    for (int r = 0; r < max_radius&&bai_num==0; r++)
    {
        // 上半圆遍历
        for (int theta_deg = 90,theta_deg2 = 90; theta_deg <= 180&&theta_deg2>=0&&bai_num==0; theta_deg++,theta_deg2--)
        {
            double theta = theta_deg * PI / 180.0;
            double theta2 = theta_deg2 * PI / 180.0;
            int x = che_x + (int)(r * sin(theta));
            int y = che_y + (int)(r * cos(theta));
            int x2 = che_x + (int)(r * sin(theta2));
            int y2 = che_y + (int)(r * cos(theta2));
            // 检查边界
            if (x >= 0 && x < HEIGHT && y >= 0 && y < WIDTH&&x2 >= 0 && x2 < HEIGHT && y2 >= 0 && y2 < WIDTH)
            {
                if (image[x][y] == WHITE)
                {
                    baidian.x=x;
                    baidian.y=y;
                    bai_num++;
                }
                else
                    if(image[x2][y2] == WHITE)
                    {
                        baidian.x=x2;
                        baidian.y=y2;
                        bai_num++;
                    }
            }
        }
    }
}
//void shibie3(uint8_t image[][MT9V03X_W], int HEIGHT, int WIDTH)
//{
//    bai_num = 0;
//    distance = 0;
//    const int max_radius = (int)sqrt((WIDTH/2)*(WIDTH/2) + (HEIGHT/2)*(HEIGHT/2)) + 1;
//    const int SUN_BRIGHTNESS_THRESH = 250;
//    const int MIN_CONTRAST = 30;
//    const int SUN_ZONE_HEIGHT = 4*HEIGHT/5; // 阳光干扰区域高度
//
//    // 第一阶段：检测并涂黑中上方的阳光干扰
//    for (int x = SUN_ZONE_HEIGHT; x < HEIGHT; x++)
//    {
//        for (int y =WIDTH/18; y <WIDTH; y++)
//        {
//            if (image[x][y] >= SUN_BRIGHTNESS_THRESH)
//            {
//                int sun_count = 0;
//                for (int dx = 0; dx <= 20; dx++)
//                {
//                    if (x+dx >= 0 && image[x+dx][y] >= SUN_BRIGHTNESS_THRESH)
//                    {
//                        sun_count++;
//                    }
//                    else
//                    {
//                        break;
//                    }
//                }
//
//                if (sun_count >= 3)
//                {
//                    for (int dx =sun_count+x;dx >=x; dx--)
//                    {
//                            for (int dy=0;dy< WIDTH;dy++)
//                            {
//                                if(image[dx][dy]>0)
//                                image[dx][dy]=0;
//                            }
//
//                    }
//                }
//            }
//        }
//    }
//
//    // 第二阶段：正常目标检测（与原逻辑相同）
//    for (int r = 0; r < max_radius && bai_num == 0; r++)
//    {
//        for (int theta_deg = 1; theta_deg <= 179 && bai_num == 0; theta_deg++)
//        {
//            double theta = theta_deg * PI / 180.0;
//            int x = che_x + (int)(r * sin(theta));
//            int y = che_y + (int)(r * cos(theta));
//
//            if (x >= 0&& x < HEIGHT*9/10 && y >=WIDTH/18 && y < WIDTH-WIDTH/18)
//            {
//                if (image[x][y] >= SUN_BRIGHTNESS_THRESH)
//                {
//                    int min_neighbor = 255;
//                    for (int dx = -2; dx <= 2; dx++)
//                    {
//                        for (int dy = -2; dy <= 2; dy++)
//                        {
//                            if (dx == 0 && dy == 0)
//                                continue;
//                            if (x+dx >= 0 && x+dx < HEIGHT && y+dy >= 0 && y+dy < WIDTH)
//                            {
//                                if (image[x+dx][y+dy] < min_neighbor)
//                                {
//                                    min_neighbor = image[x+dx][y+dy];
//                                }
//                            }
//                        }
//                    }
//                    if (image[x][y] - min_neighbor >= MIN_CONTRAST)
//                    {
//                        int neighbor_count = 0;
//                        for (int dx = -1; dx <= 1; dx++)
//                        {
//                            for (int dy = -1; dy <= 1; dy++)
//                            {
//                                if (image[x+dx][y+dy] >= SUN_BRIGHTNESS_THRESH)
//                                {
//                                    neighbor_count++;
//                                }
//                            }
//                        }
//                        if (neighbor_count >= 4)
//                        {
//                            baidian.x = x;
//                            baidian.y = y;
//                            bai_num = 1;
//                            break;
//                        }
//                    }
//                }
//            }
//        }
//    }
//}

/*
 * bai白点信息
 * 作用：解算偏转角
 */
double target_yaw=0;
double target_yaw_goal=0;//目标转弯角度
float target_yaw_last=0;//上次解算的转弯角度
void get_target_yaw(whiteaxis bai)
{
    double Tan;
    double y=che_y-bai.y;
    if(y<0)y=-y;
    if (che_x - bai.x >= 0)
    {
        Tan = ((double)che_x - bai.x) / y;
        target_yaw_goal = atan(Tan)*180/PI;
    }
    else
    {
        Tan = ((double)bai.x - che_x) / y;
        target_yaw_goal = -(atan(Tan)*180/PI);
    }
    //target_yaw=(int)target_yaw;
//    target_yaw=constrain_float(target_yaw,-55,55);
//    target_yaw =1.1*target_yaw;
    target_yaw_goal=constrain_float(target_yaw_goal,-40,40);
}

float get_distance(whiteaxis bai)
{
    float dx = bai.x - che_x;
    float dy = bai.y - che_y;
    return sqrt(dx * dx + dy * dy);
}
/*
 * 得到两点之间的距离
 */
float get_distance2(whiteaxis bai1,whiteaxis bai2)
{
    float dx = bai1.x - bai2.x;
    float dy = bai1.y - bai2.y;
    return sqrt(dx * dx + dy * dy);
}
/*!
 *  @brief      大津法二值化0.8ms程序
 *  @date:   2018-10
 *  @since      v1.2
 *  *image ：图像地址
 *  width:  图像宽
 *  height：图像高
 *  @author     Z小旋
 */
uint16 otsuThreshold(uint8 *image, uint16 width, uint16 height)
{
    #define GrayScale 256
    int pixelCount[GrayScale] = {0};//每个灰度值所占像素个数
    float pixelPro[GrayScale] = {0};//每个灰度值所占总像素比例
    int i,j;
    int Sumpix = width * height;   //总像素点
    uint8 threshold = 0;
    uint8* data = image;  //指向像素数据的指针


    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
          //   pixelCount[(int)image[i][j]]++;    若不用指针用这个
        }
    }
    float u = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / Sumpix;   //计算每个像素在整幅图像中的比例
        u += i * pixelPro[i];  //总平均灰度
    }


    float maxVariance=0.0;  //最大类间方差
    float w0 = 0, avgValue  = 0;  //w0 前景比例 ，avgValue 前景平均灰度
    for(int i = 0; i < 256; i++)     //每一次循环都是一次完整类间方差计算 (两个for叠加为1个)
    {
        w0 += pixelPro[i];  //假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
        avgValue  += i * pixelPro[i];

        float variance = pow((avgValue/w0 - u), 2) * w0 /(1 - w0);    //类间方差
        if(variance > maxVariance)
        {
            maxVariance = variance;
            threshold = i;
        }
    }


    return threshold;

}

/*
     图像二值化
     *image图像地址
     width图像宽度
     height图像高度
     threshold二值化阈值
*/
void binarization(uint8 *image,uint16 width, uint16 height,uint16 threshold)
{
    //image[h][w]
    for(int h=0;h<height;h++)
    {
        uint8 *image_temp = image + h * width;
        for(int w=0;w<width;w++)
        {
            uint8 *temp =image_temp + w;                       // 读取像素点
            if(*temp < threshold)
            {
                *temp = BLACK;
            }
            else
            {
                *temp = WHITE;
            }
        }
    }
}

/*
 * 联通域
 */
// 连通域结构体（保存区域特征）
Region  regions[200];
uint8_t labels[MT9V03X_H][MT9V03X_W];          // 标签矩阵

#define MIN_AREA 1
//#define MAX_AREA 30
#define NEAR_THRESHOLD 2
double area=0;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     二值化+连通域分析一体化函数
// 参数说明     threshold       二值化阈值
// 参数说明     regions         存储连通域特征的数组（需预分配足够空间）
// 返回参数     uint8_t         当前图像中的独立连通域数量
//-------------------------------------------------------------------------------------------------------------------
uint8_t analyze_with_binarize (uint8_t threshold, Region *regions)
{
    // 静态标签矩阵（需每次清零）

    static uint16_t labels[MT9V03X_H][MT9V03X_W];

    memset(labels, 0, sizeof(labels)); // 重置标签矩阵

    // 初始化区域数组（防止残留数据）
    memset(regions, 0, 200 * sizeof(Region)); // MAX_REGIONS需定义为足够大的值

    uint16_t current_label = 1; // 当前标签从1开始
    uint8_t region_count = 0;   // 每次调用重置计数器

    for (uint8_t y = MT9V03X_H - 1; y >= 1; y--)
    {
//    for (uint8_t y = 1; y < MT9V03X_H; y++)
//    {
        for (uint8_t x = 8; x < MT9V03X_W - 8; x++)
        {
            // 二值化处理
            uint8_t pixel = mt9v03x_image[y][x];

            if (pixel >= threshold)
            {
                // 扩展邻域检查范围
                uint16_t min_label = current_label;
                bool found = false;

                // 检查3x3邻域内的所有标签
                for (int dy = -NEAR_THRESHOLD; dy <= NEAR_THRESHOLD; dy++)
                {
                    for (int dx = -NEAR_THRESHOLD; dx <= NEAR_THRESHOLD; dx++)
                    {
                        int ny = y + dy;
                        int nx = x + dx;

                        if (nx >= 0 && nx < MT9V03X_W && ny >= 0 && ny < MT9V03X_H && labels[ny][nx] != 0)
                        {

                            if (labels[ny][nx] < min_label)
                            {
                                min_label = labels[ny][nx];
                                found = true;
                            }

                        }
                    }
                }

                // 标签分配策略
                if (found)
                {
                    labels[y][x] = min_label;
                }
                else
                {
                    labels[y][x] = current_label++;
                }

            }

        }
    }
    // 第二次扫描

    bool has_regions = false;

    for (uint8_t y = MT9V03X_H - 1; y >= 1; y--)
    {
//    for (uint8_t y = 1; y < MT9V03X_H; y++)
//    {
        for (uint8_t x = 8; x < MT9V03X_W - 8; x++)
        {
            if (labels[y][x] != 0)
            { // 处理已标记像素

                has_regions = true; // 标记存在有效区域

                uint16_t label = labels[y][x];
                uint16_t current_gray = mt9v03x_image[y][x]; // 获取原始灰度值 [[1]]

                if (label > region_count)
                {
                    // 初始化新区域
                    regions[label - 1].x = x;
                    regions[label - 1].y = y;
                    regions[label - 1].width = 1;
                    regions[label - 1].height = 1;
                    regions[label - 1].area = 1;
                    regions[label - 1].centroid_x = x;
                    regions[label - 1].centroid_y = y;

                    //考量灰度值
                    regions[label - 1].max_gray = current_gray;
                    regions[label - 1].max_gray_x = x;
                    regions[label - 1].max_gray_y = y;
                    regions[label - 1].area++;

                    whiteaxis point;
                    point.x = regions[label - 1].centroid_x;
                    point.y = regions[label - 1].centroid_y;
                    regions[label - 1].distance = get_distance(point);
                    regions[label - 1].angle = get_angle(point);

                    region_count = label;  // 更新计数器
                }
                else
                {
                    // 更新区域参数
                    // 更新边界
                    //                    if (x < regions[label-1].x) regions[label-1].x = x;
                    //                    if (y < regions[label-1].y) regions[label-1].y = y;
                    //                    regions[label-1].width = x - regions[label-1].x + 1;
                    //                    regions[label-1].height = y - regions[label-1].y + 1;
                    if (current_gray > regions[label - 1].max_gray)
                    {
                        regions[label - 1].max_gray = current_gray;
                        regions[label - 1].max_gray_x = x;
                        regions[label - 1].max_gray_y = y;
                    }

                    if (x < regions[label - 1].x)
                        regions[label - 1].x = x;
                    if (y < regions[label - 1].y)
                        regions[label - 1].y = y;
                    if (x > regions[label - 1].max_x)
                        regions[label - 1].max_x = x; // 新增max坐标维护
                    if (y > regions[label - 1].max_y)
                        regions[label - 1].max_y = y;

                    // 计算几何尺寸
                    regions[label - 1].width = regions[label - 1].max_x - regions[label - 1].x + 1;
                    regions[label - 1].height = regions[label - 1].max_y - regions[label - 1].y + 1;
                    // 更新面积和重心
                    regions[label - 1].area++;
                    regions[label - 1].centroid_x = regions[label - 1].x + regions[label - 1].width/2;

                    whiteaxis point;
                    point.x = regions[label - 1].centroid_x;
                    point.y = regions[label - 1].centroid_y;
                    regions[label - 1].distance = get_distance(point);
                    regions[label - 1].angle = get_angle(point);
                }
            }
        }
    }

    uint8_t valid_region_count = 0;
    for (uint8_t i = 0; i < region_count; i++)
    {
//        if (regions[i].area >= MIN_AREA && regions[i].area <= MAX_AREA)
        if(regions[i].y<15||(region_count>3&&regions[i].y<20))//&&regions[i].area>5
                {continue;}
                if((regions[i].x<20&&regions[i].y<20)||(regions[i].max_x&&regions[i].y<20))//&&regions[i].area>5
                        {continue;}
                if(regions[i].y>80&&regions[i].area<20)
                        {continue;}
        if (regions[i].area >= MIN_AREA)
        {
            area=regions[i].area;
            regions[valid_region_count++] = regions[i];
        }
    }

    region_count = valid_region_count;

    return has_regions ? region_count : 0;
}
/*
 *
 */
bool if_xinbiao(Region region)
{
//    uint8_t x = region.max_x - region.width / 2;
//    uint8_t y = region.max_y - region.height / 2;
    uint8_t x =region.max_gray_x;
    uint8_t y =region.max_gray_y;
    uint16_t x_max = region.max_x;
    uint16_t y_max = region.max_y;
    uint16_t x_min = region.x;
    uint16_t y_min = region.y;

    if (x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H)
    {
        return false;
    }

    bool up_valid = true;      // 向上 (y减小方向)
    bool down_valid = true;    // 向下 (y增大方向)
    bool left_valid = true;    // 向左 (x减小方向)
    bool right_valid = true;   // 向右 (x增大方向)

    uint8 current_value = mt9v03x_image[y][x];

    // 检查向上方向 (y_min)
    for (int i = y - 1; i >= y_min; i--)
    {
        if (mt9v03x_image[i][x] > current_value)
        {
            up_valid = false;
            break;
        }
        current_value = mt9v03x_image[i][x];
    }

    current_value = mt9v03x_image[y][x];
    // 检查向下方向 (y_max)
    for (int i = y + 1; i < y_max; i++)
    {
        if (mt9v03x_image[i][x] > current_value)
        {
            down_valid = false;
            break;
        }
        current_value = mt9v03x_image[i][x];
    }

    current_value = mt9v03x_image[y][x];
    // 检查向左方向 (x_min)
    for (int j = x - 1; j >= x_min; j--)
    {
        if (mt9v03x_image[y][j] > current_value)
        {
            left_valid = false;
            break;
        }
        current_value = mt9v03x_image[y][j];
    }

    current_value = mt9v03x_image[y][x];
    // 检查向右方向 (x_max)
    for (int j = x + 1; j < x_max; j++)
    {
        if (mt9v03x_image[y][j] > current_value)
        {
            right_valid = false;
            break;
        }
        current_value = mt9v03x_image[y][j];
    }

    // 返回四个方向是否都满足条件
    return up_valid && down_valid && left_valid && right_valid;
}
/*
 * 获取距离车最近的连通域下标
 * regions[]联通域数组
 * region_count联通域数量
 * return 最近下标
 */
//uint8_t zuijin_xiabiao (Region regions[],uint8_t region_count)
//{
//    float distance_min = 9999;
//    uint8_t xiabiao = 200;
//    for (uint8_t i = 0; i < region_count; i++)
//    {
//        whiteaxis point;
//        point.x = regions[i].centroid_x;
//        point.y = regions[i].centroid_y;
//        float distance = get_distance(point);
//        if (distance < distance_min && regions[i].area > 2 )
//        {
//            distance_min = distance;
//            xiabiao = i;
//        }
//    }
//
//    return xiabiao;
//}
/*
 * 获取距离车最近的连通域下标
 * regions[]联通域数组
 * region_count联通域数量
 * return 最近下标
 */
uint8_t zuijin_xiabiao (Region regions[], uint8_t region_count)
{
    float distance_min = 9999;
    uint8_t xiabiao = region_count;
    for (uint8_t i = 0; i < region_count; i++)//找最近连通域下标
    {
        float distance = regions[i].distance;
        if (distance < distance_min)
        {
            distance_min = distance;
            xiabiao = i;
        }
    }
    if (xiabiao < region_count - 1 && xiabiao > 0)//当最近的连通域不是第一个或者最后一个，判断连通域相邻两个连通域是否有更合适的
    {
        uint8_t xiabiao2, xiabiao3;
        bool flag2 = false, flag3 = false;
        if (fabs(regions[xiabiao].distance-regions[xiabiao-1].distance) < 5)//与上一个连通域距离差小于5时
        {
            if (fabs(regions[xiabiao].angle) <= fabs(regions[xiabiao - 1].angle))//选择与车的夹角最小的
            {
                xiabiao2 = xiabiao;
            }
            else
            {
                xiabiao2 = xiabiao - 1;
            }
            xiabiao = xiabiao2;
            flag2 = true;
        }
        if (fabs(regions[xiabiao].distance-regions[xiabiao+1].distance) < 5)//与下一个连通域距离差小于5时
        {
            if (fabs(regions[xiabiao].angle) <= fabs(regions[xiabiao + 1].angle))//选择与车的夹角最小的
            {
                xiabiao3 = xiabiao;
            }
            else
            {
                xiabiao3 = xiabiao + 1;
            }
            xiabiao = xiabiao3;
            flag3 = true;
        }
        if (flag2 && flag3)//从“上一个连通域”和“下一个联通域”中选择夹角更小的
        {
            if (fabs(regions[xiabiao2].angle) <= fabs(regions[xiabiao3].angle))
            {
                xiabiao = xiabiao2;
            }
            else
            {
                xiabiao = xiabiao3;
            }
        }
    }
    else if (xiabiao == 0 && region_count > 1)   //是第一个联通域时
    {
        if (fabs(regions[xiabiao].distance-regions[xiabiao+1].distance) < 5)
        {
            if (fabs(regions[xiabiao].angle) <= fabs(regions[xiabiao + 1].angle))
            {
            }
            else
            {
                xiabiao = xiabiao + 1;
            }
        }
    }
    else if (xiabiao == region_count - 1 && region_count > 1)   //是最后一个连通域时
    {
        if (fabs(regions[xiabiao].distance-regions[xiabiao-1].distance) < 5)
        {
            if (fabs(regions[xiabiao].angle) <= fabs(regions[xiabiao - 1].angle))
            {
            }
            else
            {
                xiabiao = xiabiao - 1;
            }
        }
    }

    return xiabiao;
}
//大jing法求阈值

double get_angle(whiteaxis bai)
{
    double Tan;
    double y=che_y-bai.y;
    double angle;
    if(y<0)y=-y;
    if (che_x - bai.x >= 0)
    {
        Tan = ((double)che_x - bai.x) / y;
        angle = atan(Tan)*180/PI;
    }
    else
    {
        Tan = ((double)bai.x - che_x) / y;
        angle = -(atan(Tan)*180/PI);
    }
    return angle;
}

