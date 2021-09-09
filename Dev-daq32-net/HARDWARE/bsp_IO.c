/*****************************************************************************
    bsp_IO.c
指示灯、按键、蜂鸣器所用到IO初始化
******************************************************************************/

#include "bsp_io.h"   

/**********************************************************
    IO_GPIO_Config
描述：所用到GPIO初始化
**********************************************************/
void IO_GPIO_Config(void)
{       
    /*定义一个GPIO_InitTypeDef类型的结构体*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /*开启相关的GPIO外设时钟*/
    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOF, ENABLE); 
    //输出口
    /*设置引脚模式为输出模式*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
    /*设置引脚的输出类型为推挽输出*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
    /*设置引脚为上拉模式*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    /*设置引脚速率为50MHz */   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

    // LED1~5															   
    GPIO_InitStructure.GPIO_Pin = LED1_PIN; 
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure); 

    GPIO_InitStructure.GPIO_Pin = LED2_PIN; 
    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure); 

    GPIO_InitStructure.GPIO_Pin = LED3_PIN; 
    GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure); 

    GPIO_InitStructure.GPIO_Pin = LED4_PIN; 
    GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStructure); 

    GPIO_InitStructure.GPIO_Pin = LED5_PIN; 
    GPIO_Init(LED5_GPIO_PORT, &GPIO_InitStructure); 

    //BELL
    GPIO_InitStructure.GPIO_Pin = BELL_PIN; 
    GPIO_Init(BELL_GPIO_PORT, &GPIO_InitStructure); 

    //输入口
    // 设置引脚为输入模式
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
    // 设置引脚上拉
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

    //Key1~3
    GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
    GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);   

    GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
    GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);   

    GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
    GPIO_Init(KEY3_GPIO_PORT, &GPIO_InitStructure);   

    //IO口初始态
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();       
    LED4_OFF();
    LED5_OFF();
    BELL_OFF();
}
/*********************************************END OF FILE**********************/
