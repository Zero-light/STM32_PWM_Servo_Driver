#include "ZHUIDIAN.h"
#include "stm32f10x.h" 
#include "PWM.h"
#include "zbsjcll.h"
#include "Delay.h"
#include <math.h>
#include <stdio.h>
float enX;
float enY;
DUOJI SH;
DUOJI XIA;

// 下舵机角度增量控制（X轴）
void AddAngle_X(float incAngle) {
    static float currentAngleX = 90.0f;  // 初始角度
    currentAngleX += incAngle;  // 注意方向是否正确，不正确则改为+=
    
    // 限制舵机角度范围
    if (currentAngleX > 270.0f) currentAngleX = 270.0f;
    if (currentAngleX < 0.0f) currentAngleX = 0.0f;
    
    // 转换为PWM输出（500-2500对应0-270度）
    PWM_SetCompare2((uint16_t)(currentAngleX / 270.0f * 2000.0f + 500.0f));
}

// 上舵机角度增量控制（Y轴）
void AddAngle_Y(float incAngle) {
    static float currentAngleY = 90.0f;  // 初始角度
    currentAngleY += incAngle;  // 注意方向是否正确，不正确则改为+=
    
    // 限制舵机角度范围
    if (currentAngleY > 180.0f) currentAngleY = 180.0f;
    if (currentAngleY < 0.0f) currentAngleY = 0.0f;
    
    // 转换为PWM输出
    PWM_SetCompare1((uint16_t)(currentAngleY / 180.0f * 2000.0f + 500.0f));
}


void ExecutePath(float endX, float endY) {
     enX = endX;
     enY = endY;  
	
}
    
  

		
