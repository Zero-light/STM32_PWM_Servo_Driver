#include "Timer.h"
#include "stm32f10x.h" 
#include "PWM.h"
#include "zbsjcll.h"
#include "Delay.h"
#include <math.h>

// PID参数初始化（下舵机）
DUOJI_PID XIA_PID = {
    .Kp = 0.8f,
    .Ki = 0.03f,
    .Kd = 0.25f,
    .OutMax = 4.0f,
    .OutMin = -4.0f,
    .TotalMax = 40.0f,
    .TotalMin = -40.0f,
    .DeadBand = 2.0f,
    .SeparateBand = 15.0f,
    .KdFilter = 0.7f,
    .IntegralMax = 5.0f,
    .IntegralMin = -5.0f,
    .KpScale = 1.0f,
    .KiScale = 1.0f
};

// PID参数初始化（上舵机）
DUOJI_PID SH_PID = {
		.Error0=0.0f,
		.Error1=0.0f,
		.Error2=0.0f,	
    .Kp = 0.8f,		//比例系数，主要响应当前误差
    .Ki = 0.03f,		//积分系数，用于消除稳态误差
    .Kd = 0.25f,		//微分系数，用于抑制超调和改善动态响应
    .OutMax = 4.0f,
    .OutMin = -4.0f,			//输出限幅（4.0/-4.0），限制 PID 总输出范围
    .TotalMax = 40.0f,
    .TotalMin = -40.0f,			//积分限幅，最大积分数	
    .DeadBand = 2.0f,			//死区范围，当误差小于此值时不进行调节
    .SeparateBand = 15.0f,				//分段调节的阈值，可能用于在不同误差范围使用不同控制策略
    .KdFilter = 0.7f,		//微分滤波系数，用于平滑微分环节的输出
    .IntegralMax = 5.0f,
    .IntegralMin = -5.0f,			//积分限幅（5.0/-5.0），防止积分饱和
    .KpScale = 1.0f,
    .KiScale = 1.0f			//比例和积分的缩放系数（均为 1.0，即不缩放）
};

// 优化的增量式PID计算（带动态参数和抗饱和）
float Incremental_PID_Calculate(DUOJI_PID *pid) {
    // 计算当前误差
    pid->Error0 = pid->Target - pid->Actual;

    // 死区带滞后处理
    if (fabs(pid->Error0) <= pid->DeadBand) {
        if (pid->IncrOut == 0 || fabs(pid->Error0) <= pid->DeadBand * 0.8f) {
            pid->IncrOut = 0;
            return 0;
        }
    }

    // 动态参数缩放（核心减速机制）
    float errorAbs = fabs(pid->Error0);
    pid->KpScale = (errorAbs > 30.0f) ? 1.2f :  // 远：快速响应
                   (errorAbs < 10.0f) ? 0.3f :  // 近：大幅减速
                   1.0f;                        // 中：正常速度
    pid->KiScale = (errorAbs > 20.0f) ? 0.1f :  // 远：弱积分
                   (errorAbs < 5.0f) ? 1.5f :   // 近：强积分消静差
                   0.5f;

    // 积分分离与积分限幅
    float integral = 0;
    if (errorAbs <= pid->SeparateBand) {
        integral = pid->Ki * pid->KiScale * pid->Error0;
        integral = (integral > pid->IntegralMax) ? pid->IntegralMax : integral;
        integral = (integral < pid->IntegralMin) ? pid->IntegralMin : integral;
    }

    // 微分滤波（减少振荡）
    float D_raw = pid->Kd * (pid->Error0 - 2 * pid->Error1 + pid->Error2);
    pid->DLast = pid->KdFilter * D_raw + (1 - pid->KdFilter) * pid->DLast;

    // 计算增量输出
    pid->IncrOut = (pid->Kp * pid->KpScale) * (pid->Error0 - pid->Error1) +
                   integral +
                   pid->DLast;

    // 单次增量限幅
    if (pid->IncrOut > pid->OutMax) pid->IncrOut = pid->OutMax;
    else if (pid->IncrOut < pid->OutMin) pid->IncrOut = pid->OutMin;

    // 总输出限幅与抗饱和
    pid->TotalOut += pid->IncrOut;
    if (pid->TotalOut > pid->TotalMax) {
        pid->TotalOut = pid->TotalMax;
        if (integral > 0) integral = 0;
    } else if (pid->TotalOut < pid->TotalMin) {
        pid->TotalOut = pid->TotalMin;
        if (integral < 0) integral = 0;
    }

    // 保存误差历史
    pid->Error2 = pid->Error1;
    pid->Error1 = pid->Error0;

    return pid->IncrOut;
}

// 下舵机角度增量控制
void Servo_AddAngle_X(float incAngle) {
    static float currentAngleX = 90.0f;  // 初始角度
    currentAngleX += incAngle;
    
    // 限制舵机角度范围
    if (currentAngleX > 180.0f) currentAngleX = 180.0f;
    if (currentAngleX < 0.0f) currentAngleX = 0.0f;
    
    // 转换为PWM输出（500-2500对应0-180度）
    PWM_SetCompare2(currentAngleX / 180.0f * 2000.0f + 500.0f);
}

// 上舵机角度增量控制
void Servo_AddAngle_Y(float incAngle) {
    static float currentAngleY = 90.0f;  // 初始角度
    currentAngleY += incAngle;
    
    // 限制舵机角度范围
    if (currentAngleY > 180.0f) currentAngleY = 180.0f;
    if (currentAngleY < 0.0f) currentAngleY = 0.0f;
    
    // 转换为PWM输出
    PWM_SetCompare1(currentAngleY / 180.0f * 2000.0f + 500.0f);
}

// 从当前位置直线移动到指定终点（主函数调用此函数）
void ExecutePath(float endX, float endY) {
    // 1. 获取当前位置作为起点（摄像头实时反馈）
    float startX = Camera_GetX();
    float startY = Camera_GetY();
    
    // 2. 计算总位移
    float deltaX = endX - startX;
    float deltaY = endY - startY;
    
    // 3. 分10个目标点离散化路径（确保直线运动）
    ZUOBIAO waypoints[10];  // 中间路径点
    for (uint8_t i = 0; i < 10; i++) {
        waypoints[i].X = startX + deltaX * (i + 1) / 10.0f;  // 按比例分配X坐标
        waypoints[i].Y = startY + deltaY * (i + 1) / 10.0f;  // 按比例分配Y坐标（保证直线）
    }
    
    // 4. 依次移动到每个中间点（带平滑过渡）
    float lastX = startX;
    float lastY = startY;
    for (uint8_t i = 0; i < 10; i++) {
        float targetX = waypoints[i].X;
        float targetY = waypoints[i].Y;
        
        // 每个中间点分5步减速逼近（二次平滑）
        for (uint8_t step = 1; step <= 5; step++) {
            // 计算当前过渡目标
            XIA_PID.Target = lastX + (targetX - lastX) * step / 5.0f;
            SH_PID.Target = lastY + (targetY - lastY) * step / 5.0f;
            
            // 控制到过渡目标
            uint32_t timeout = 0;
            while (timeout < 5) {  // 超时保护（最多0.25秒）
                // 更新摄像头反馈
                XIA_PID.Actual = Camera_GetX();
                SH_PID.Actual = Camera_GetY();
                
                // PID计算并驱动舵机
                float angleIncX = Incremental_PID_Calculate(&XIA_PID);
                float angleIncY = Incremental_PID_Calculate(&SH_PID);
                Servo_AddAngle_X(angleIncX);
                Servo_AddAngle_Y(angleIncY);
                
                // 到达过渡目标则进入下一步
                if (fabs(XIA_PID.Target - XIA_PID.Actual) <= XIA_PID.DeadBand * 1.2f &&
                    fabs(SH_PID.Target - SH_PID.Actual) <= SH_PID.DeadBand * 1.2f) {
                    break;
                }
                
                Delay_ms(50);  // 50ms控制周期
                timeout++;
            }
        }
        
        // 更新上一个点为当前中间点
        lastX = targetX;
        lastY = targetY;
    }
}
   
