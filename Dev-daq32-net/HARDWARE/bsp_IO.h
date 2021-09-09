#ifndef __IO_H
#define	__IO_H

#include "stm32f4xx.h"

//指示灯、按键、蜂鸣器所用到IO引脚定义
/*******************************************************/
//LED
#define LED1_PIN                GPIO_Pin_0                 
#define LED1_GPIO_PORT          GPIOF                      
#define LED1_GPIO_CLK           RCC_AHB1Periph_GPIOF

#define LED2_PIN                GPIO_Pin_1                 
#define LED2_GPIO_PORT          GPIOF                      
#define LED2_GPIO_CLK           RCC_AHB1Periph_GPIOF

#define LED3_PIN                GPIO_Pin_2                 
#define LED3_GPIO_PORT          GPIOF                      
#define LED3_GPIO_CLK           RCC_AHB1Periph_GPIOF

#define LED4_PIN                GPIO_Pin_3                 
#define LED4_GPIO_PORT          GPIOF                      
#define LED4_GPIO_CLK           RCC_AHB1Periph_GPIOF

#define LED5_PIN                GPIO_Pin_4                
#define LED5_GPIO_PORT          GPIOF                     
#define LED5_GPIO_CLK           RCC_AHB1Periph_GPIOF

//蜂鸣器
#define BELL_PIN                GPIO_Pin_7                 
#define BELL_GPIO_PORT          GPIOC                      
#define BELL_GPIO_CLK           RCC_AHB1Periph_GPIOC

//按键输入
#define KEY1_PIN                GPIO_Pin_14                
#define KEY1_GPIO_PORT          GPIOB                      
#define KEY1_GPIO_CLK           RCC_AHB1Periph_GPIOB

#define KEY2_PIN                GPIO_Pin_15                
#define KEY2_GPIO_PORT          GPIOB                      
#define KEY2_GPIO_CLK           RCC_AHB1Periph_GPIOB

#define KEY3_PIN                GPIO_Pin_10               
#define KEY3_GPIO_PORT          GPIOB                      
#define KEY3_GPIO_CLK           RCC_AHB1Periph_GPIOB

// 定义控制IO的宏  
// LED
#define LED1_TOGGLE()		    digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF()		        digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON()				digitalLo(LED1_GPIO_PORT,LED1_PIN)

#define LED2_TOGGLE()		    digitalToggle(LED2_GPIO_PORT,LED2_PIN)
#define LED2_OFF()		        digitalHi(LED2_GPIO_PORT,LED2_PIN)
#define LED2_ON()				digitalLo(LED2_GPIO_PORT,LED2_PIN)

#define LED3_TOGGLE()		    digitalToggle(LED3_GPIO_PORT,LED3_PIN)
#define LED3_OFF()		        digitalHi(LED3_GPIO_PORT,LED3_PIN)
#define LED3_ON()				digitalLo(LED3_GPIO_PORT,LED3_PIN)

#define LED4_TOGGLE()		    digitalToggle(LED4_GPIO_PORT,LED4_PIN)
#define LED4_OFF()		        digitalHi(LED4_GPIO_PORT,LED4_PIN)
#define LED4_ON()				digitalLo(LED4_GPIO_PORT,LED4_PIN)

#define LED5_TOGGLE()		    digitalToggle(LED5_GPIO_PORT,LED5_PIN)
#define LED5_OFF()		        digitalHi(LED5_GPIO_PORT,LED5_PIN)
#define LED5_ON()				digitalLo(LED5_GPIO_PORT,LED5_PIN)

//蜂鸣器
#define BELL_ON()		        digitalHi(BELL_GPIO_PORT,BELL_PIN)
#define BELL_OFF()			    digitalLo(BELL_GPIO_PORT,BELL_PIN)

//按键输入
#define IS_KEY1(x)             ((x)==GPIO_ReadInputDataBit(KEY1_GPIO_PORT,KEY1_PIN))
#define IS_KEY2(x)             ((x)==GPIO_ReadInputDataBit(KEY2_GPIO_PORT,KEY2_PIN))
#define IS_KEY3(x)             ((x)==GPIO_ReadInputDataBit(KEY3_GPIO_PORT,KEY3_PIN))

//----------------------------------
void IO_GPIO_Config(void);

#endif /* __IO_H */
