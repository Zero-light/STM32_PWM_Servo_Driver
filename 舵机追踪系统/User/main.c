#include "stm32f10x.h"                  
#include "ZHUIDIAN.h"       // 包含舵机控制相关声明
#include "zbsjcll.h" 
#include "Serial.h" 
#include "PWM.h" 
#include "Delay.h"
#include "OLED.h" 
#include "Timer.h"

// 外部声明舵机变量（确保在main中可见）
extern DUOJI XIA;
extern DUOJI SH;

int main(void) {
    // 初始化外设
    PWM_Init();
    OLED_Init();
    Serial_Init();
    Timer1_Init();
    // 初始化舵机到中间位置X是PA1
    AddAngle_X(0);
    AddAngle_Y(0);
    Delay_ms(2000);  // 等待舵机稳定
		OLED_ShowString(3, 1, "88888");
    // 主循环：持续追踪中心点(320,240)
    while (1) {
        OLED_ShowString(1, 1, "X:");
        OLED_ShowNum(1, 3, (uint16_t)XIA.Actual, 3);
        OLED_ShowString(2, 1, "Y:");
        OLED_ShowNum(2, 3, (uint16_t)SH.Actual, 3);
			    
    }
}

