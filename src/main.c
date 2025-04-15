




#include "stm32f10x.h"
#include "lcd.h"
#include "stm32f10x_adc.h"
#include "pwm.h"
#include "key.h"

// 系统参数结构体
typedef struct {
    float voltage;
    float current;
    float set_voltage;
    float set_current;
    uint8_t power_level;
    uint8_t charging_enabled;
    uint8_t discharging_enabled;
} SystemState;

volatile SystemState sys_state = {0};
uint8_t display_refresh_flag = 1;

// 界面布局定义
#define STATUS_BAR_HEIGHT  40
#define PARAM_PANEL_Y      50
#define VALUE_COLUMN       120

// 颜色定义
#define PANEL_BG    0x18E3
#define WARNING_RED 0xF800
#define SAFE_GREEN  0x07E0

void System_Init(void);
void Update_Display(void);
void Process_Inputs(void);
void Charge_Control(void);

int main(void) {
    System_Init();
    LCD7789_FillScreen(BLACK);
    
    while(1) {
        // 1. 数据采集
        sys_state.voltage = ADC_GetVoltage();
        sys_state.current = ADC_GetCurrent();
        
        // 2. 状态计算
        sys_state.power_level = (uint8_t)((sys_state.voltage - 3.0) / (4.2 - 3.0) * 100);
        
        // 3. 控制逻辑
        Charge_Control();
        
        // 4. 显示更新
        if(display_refresh_flag) {
            Update_Display();
            display_refresh_flag = 0;
        }
        
        // 5. 处理输入
        Process_Inputs();
        
        Delay_ms(100);
    }
}

// 系统初始化
void System_Init(void) {
    // 硬件初始化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    LCD7789_Init();
    ADC_Init();
    PWM_Init();
    KEY_Init();
    
    // 默认参数设置
    sys_state.set_voltage = 4.2;    // 默认充电截止电压
    sys_state.set_current = 1.0;    // 默认充电电流
    sys_state.charging_enabled = 1;
}

// 充电控制逻辑
void Charge_Control(void) {
    static uint8_t charging = 0;
    
    if(sys_state.charging_enabled) {
        if(sys_state.voltage < sys_state.set_voltage) {
            PWM_SetDuty(80);  // 80%占空比充电
            charging = 1;
        } else {
            PWM_SetDuty(0);
            charging = 0;
        }
    }
    
    // 更新状态标志
    sys_state.power_level = charging ? sys_state.power_level+1 : sys_state.power_level-1;
    sys_state.power_level = (sys_state.power_level > 100) ? 100 : 
                           (sys_state.power_level < 0) ? 0 : sys_state.power_level;
}

// 显示界面更新
void Update_Display(void) {
    char buf[30];
    
    // 1. 状态栏
    LCD7789_FillRect(0, 0, LCD_WIDTH, STATUS_BAR_HEIGHT, PANEL_BG);
    LCD7789_DrawString(10, 10, "BMS Monitor v1.0", WHITE, PANEL_BG, 2);
    
    // 2. 参数显示面板
    LCD7789_DrawRect(5, PARAM_PANEL_Y, LCD_WIDTH-10, 150, CYAN);
    
    // 电压显示
    sprintf(buf, "Voltage: %.2fV", sys_state.voltage);
    LCD7789_DrawString(10, PARAM_PANEL_Y+20, buf, WHITE, BLACK, 1);
    
    // 电流显示（颜色指示）
    uint16_t current_color = (fabs(sys_state.current) > sys_state.set_current) ? WARNING_RED : SAFE_GREEN;
    sprintf(buf, "Current: %+.2fA", sys_state.current);
    LCD7789_DrawString(10, PARAM_PANEL_Y+50, buf, current_color, BLACK, 1);
    
    // 电量条
    LCD7789_DrawRect(10, PARAM_PANEL_Y+80, 200, 20, WHITE);
    LCD7789_FillRect(10, PARAM_PANEL_Y+80, sys_state.power_level*2, 20, BLUE);
    sprintf(buf, "%d%%", sys_state.power_level);
    LCD7789_DrawString(220, PARAM_PANEL_Y+80, buf, WHITE, BLACK, 1);
    
    // 状态指示
    LCD7789_DrawCircle(LCD_WIDTH-40, 20, 15, sys_state.charging_enabled ? GREEN : RED);
}

// 按键处理
void Process_Inputs(void) {
    static uint8_t last_key = 0;
    uint8_t current_key = KEY_Scan(0);
    
    if(current_key && (current_key != last_key)) {
        switch(current_key) {
            case KEY_UP:
                sys_state.set_voltage += 0.1;
                if(sys_state.set_voltage > 4.2) sys_state.set_voltage = 4.2;
                break;
                
            case KEY_DOWN:
                sys_state.set_voltage -= 0.1;
                if(sys_state.set_voltage < 3.0) sys_state.set_voltage = 3.0;
                break;
                
            case KEY_OK:
                sys_state.charging_enabled ^= 1;
                break;
        }
        display_refresh_flag = 1;
    }
    last_key = current_key;
}



void ADC_Init(void) {
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能ADC1时钟和GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    // 配置ADC通道0（PA0）为模拟输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // ADC参数配置
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // 校准并启用ADC
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

float ADC_GetVoltage(void) {
    // 假设电压检测分压比为1/4，参考电压3.3V
    return (ADC_GetConversionValue(ADC1) * 3.3 / 4096) * 4;
}

float ADC_GetCurrent(void) {
    // 假设电流检测使用0.1Ω电阻，运放增益50
    return (ADC_GetConversionValue(ADC2) * 3.3 / 4096 - 1.65) / 0.1 / 50;
}

// 在stm32f10x_conf.h中确保包含必要的外设库
#define USE_STDPERIPH_DRIVER
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"

// PWM配置（以TIM2为例）
void PWM_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 时钟使能
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 时基配置：1kHz PWM
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71; // 72MHz/72=1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // PWM通道配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始占空比0%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

void PWM_SetDuty(uint8_t duty) {
    TIM2->CCR1 = (duty * 1000) / 100; // 0-100%对应0-1000
}



// 在stm32f10x.h中开启优化
#pragma GCC optimize ("O3")

// 使用DMA加速显示刷新
void LCD7789_DMA_Update(uint16_t *buffer, uint32_t size) {
    // 实现DMA传输显示数据
}

// 在Charge_Control()中添加保护
if(sys_state.voltage > 4.25) {
    PWM_SetDuty(0);
    sys_state.charging_enabled = 0;
    LCD7789_DrawString(10, 170, "OVER VOLTAGE!", WARNING_RED, BLACK, 2);
}



void Touch_Handler(void) {
    if(Touch_GetXY(&x, &y)) {
        if(x > 200 && y < 40) { // 点击状态栏
            sys_state.charging_enabled ^= 1;
            display_refresh_flag = 1;
        }
    }
}

#ifndef __PWM_H
#define __PWM_H



// PWM通道定义
#define PWM_TIM               TIM2
#define PWM_TIM_CLK           RCC_APB1Periph_TIM2
#define PWM_GPIO_CLK          RCC_APB2Periph_GPIOA
#define PWM_GPIO_PORT         GPIOA
#define PWM_GPIO_PIN          GPIO_Pin_1  // TIM2_CH2
#define PWM_GPIO_PIN_SOURCE   GPIO_PinSource1
#define PWM_AF               GPIO_AF_TIM2

// 函数声明
void PWM_Init(void);
void PWM_SetDuty(uint8_t duty);  // duty: 0-100%

#endif /* __PWM_H */
pwm.c
c
复制
#include "pwm.h"

void PWM_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 1. 时钟使能
    RCC_APB1PeriphClockCmd(PWM_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(PWM_GPIO_CLK, ENABLE);
    
    // 2. GPIO配置：复用推挽输出
    GPIO_InitStructure.GPIO_Pin = PWM_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStructure);
    
    // 3. 时基配置：10kHz PWM
    TIM_TimeBaseStructure.TIM_Period = 999;          // ARR值
    TIM_TimeBaseStructure.TIM_Prescaler = 71;        // 72MHz/(71+1)=1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStructure);
    
    // 4. PWM通道配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;               // 初始占空比0%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);      // 使用通道2
    
    // 5. 启动PWM
    TIM_Cmd(PWM_TIM, ENABLE);
    TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
}

void PWM_SetDuty(uint8_t duty) {
    if(duty > 100) duty = 100;
    PWM_TIM->CCR2 = (duty * (PWM_TIM->ARR + 1)) / 100;
}




// 初始化
PWM_Init();
KEY_Init();

// 主循环中处理按键
uint8_t key = KEY_Scan(0);
if(key == KEY_UP) {
    sys_state.set_voltage += 0.1;
    display_refresh_flag = 1;
}

// 控制充放电
if(sys_state.charging_enabled) {
    PWM_SetDuty(80);  // 80%占空比充电
} else {
    PWM_SetDuty(0);    // 关闭输出
}
