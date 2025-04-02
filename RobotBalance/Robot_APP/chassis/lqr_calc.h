#include "balance.h"
#include "stdint.h"
#include "arm_math.h"


// float LQR_K[12] = {0};

/**
 * @brief 根据状态反馈计算当前腿长,查表获得LQR的反馈增益,并列式计算LQR的输出
 * @note 得到的腿部力矩输出还要经过综合运动控制系统补偿后映射为两个关节电机输出
 *
 */
static void CalcLQR(LinkNPodParam *p, ChassisParam *chassis)
{

static float k[12][3] = {116.081683,-143.750546,-4.631389,
-1.716839,-18.221274,0.613814,
30.038309,-23.019396,-2.052031,
38.973772,-33.006676,-2.652186,
145.177829,-138.317495,43.674086,
15.170052,-14.969698,4.842460,
-23.184244,-6.686067,23.108112,
-13.042988,10.541006,1.713458,
43.387476,-42.777318,14.461913,
52.766447,-52.658226,18.797175,
-301.558090,230.714114,37.542662,
-29.584037,22.596265,1.791558}; 

    float T[2] = {0};      // 0 T_wheel  1 T_hip
    float l = p->leg_len;
    float lsqr = l * l;
    float lsqr3 = l * l * l;

    uint8_t i, j;


    // p->theta_w = Low_pass_filter(&chasiss_vel_lowpassfilter,p->theta_w);
    // 离地时轮子输出置0
    i = 0; j = i * 6;
    T[i] = p->fly_flag ? 0 :
           ((k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) * -p->theta +
            (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) * -p->theta_w + 
            (k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * (chassis->target_dist - chassis->dist) +
            (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * (chassis->target_v - chassis->vel) +
            (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -chassis->pitch +
            (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -chassis->pitch_w);

    i = 1; j = i * 6;
    T[i] = (k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) *  -p->theta +
           (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) *  -p->theta_w + (p->fly_flag ? 0 :
           ((k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * (chassis->target_dist - chassis->dist) +
            (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * (chassis->target_v - chassis->vel) +
            (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -chassis->pitch +
            (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -chassis->pitch_w));	

    
    p->T_wheel = T[0];
    p->T_hip = -T[1];
}



