#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

// 坐标结构体（单位：cm）
typedef struct {
    float X;
    float Y;
} ZUOBIAO;

// 优化后的PID结构体
typedef struct {
    float Target;        // 目标位置（cm）
    float Actual;        // 实际位置（摄像头反馈）
    float Kp;            // 基础比例系数
    float Ki;            // 基础积分系数
    float Kd;            // 基础微分系数
    float Error0;        // 当前误差
    float Error1;        // 上一次误差
    float Error2;        // 上上次误差
    float IncrOut;       // 增量输出
    float TotalOut;      // 总输出
    float OutMax;        // 单次增量上限
    float OutMin;        // 单次增量下限
    float TotalMax;      // 总输出上限
    float TotalMin;      // 总输出下限
    float DeadBand;      // 死区（cm）
    float SeparateBand;  // 积分分离阈值
    float KdFilter;      // 微分滤波系数
    float DLast;         // 滤波后的微分项
    float IntegralMax;   // 积分项上限
    float IntegralMin;   // 积分项下限
    float KpScale;       // Kp动态缩放系数
    float KiScale;       // Ki动态缩放系数
} DUOJI_PID;

// 外部变量声明
extern DUOJI_PID XIA_PID;  // 下舵机PID
extern DUOJI_PID SH_PID;   // 上舵机PID

// 函数声明
float Incremental_PID_Calculate(DUOJI_PID *pid);
void Servo_AddAngle_X(float incAngle);
void Servo_AddAngle_Y(float incAngle);
void ExecutePath(float endX, float endY);  // 主函数调用的路径执行函数

#endif
    
