#ifndef ZHUIDIAN_H
#define ZHUIDIAN_H

#include "stm32f10x.h"

// 舵机结构体定义
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float OutMax;       // 单次输出最大值
    float OutMin;       // 单次输出最小值
    float DeadBand;     // 死区范围（坐标单位）
    float SeparateBand; // 积分分离阈值（坐标单位）
    float IntegralMax;  // 积分最大值
    float IntegralMin;  // 积分最小值
    float KpScale;      // 比例缩放系数
    float KiScale;      // 积分缩放系数
    float Target;       // 目标坐标
    float Actual;       // 实际坐标（滤波后）
    float Error0;       // 当前误差
    float Error1;       // 上一误差
    float Error2;       // 上两误差
    float Out;          // 输出值（坐标单位）
    float Integral;     // 积分值
    float AngleRatio;   // 坐标到角度的转换比例
} DUOJI;

// 坐标结构体定义
typedef struct {
    float X;
    float Y;
} ZUOBIAO;

// 坐标-角度转换比例 (1单位坐标对应的舵机角度变化)
// 根据实际测试调整，1度对应10-20单位坐标，则1单位坐标约对应0.05-0.1度
#define COORD_TO_ANGLE_X 0.4f  // X轴转换系数
#define COORD_TO_ANGLE_Y 0.4f  // Y轴转换系数

// 滑动平均滤波（处理摄像头数据噪声）
#define FILTER_DEPTH 5
void ExecutePath(float endX, float endY);//追踪点函数
void AddAngle_Y(float incAngle);//输出增量角度函数，将想要输出的增量角度输入，函数转化为pwm波累加输出
void AddAngle_X(float incAngle);
float PID(DUOJI*pid);//带入结构体，进行PID得出应增量的角度值
extern DUOJI XIA;
extern DUOJI SH;
extern float enX;
extern float enY;
#endif

