/************************************************* 
    AD7606���� 
16bit,���� 
����PWM�����ΪAD�����ź�Դ��ʵ���Զ����������� 
ת�����ͨ���ⲿ�ж�֪ͨCPU 
*************************************************/
#ifndef __AD7606_H
#define __AD7606_H															    

#include "stm32f4xx.h"
#include "main.h"
//--------------------------------------
//PWM1 GPIO �궨��
#define OCPWM1_PIN          GPIO_Pin_0              
#define OCPWM1_GPIO_PORT    GPIOA                      
#define OCPWM1_GPIO_CLK     RCC_AHB1Periph_GPIOA
#define OCPWM1_PinSource	GPIO_PinSource0
#define OCPWM1_AF			GPIO_AF_TIM5
//PWM�ö�ʱ��
#define OCPWM1_TIM          TIM5
#define OCPWM1_TIM_CLK      RCC_APB1Periph_TIM5
//ͨ�ÿ��ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK/2=84MHz
//����Ƶ��=84000K/((Prescaler+1)*(Period+1)) 
//TIM_Period����ֵ(����ʱ�Ӳ���Ƶ����Prescaler=0)
#define T200K       420      
#define T100K       840      
#define T80K        1050
#define T50K        1680
#define T20K        4200 
 
//--------------------------------------

// D �����ź��� 
#define AD7606_D0_GPIO_PORT        GPIOD
#define AD7606_D0_GPIO_CLK         RCC_AHB1Periph_GPIOD
#define AD7606_D0_GPIO_PIN         GPIO_Pin_14
#define AD7606_D0_GPIO_PinSource   GPIO_PinSource14

#define AD7606_D1_GPIO_PORT        GPIOD
#define AD7606_D1_GPIO_CLK         RCC_AHB1Periph_GPIOD
#define AD7606_D1_GPIO_PIN         GPIO_Pin_15
#define AD7606_D1_GPIO_PinSource   GPIO_PinSource15

#define AD7606_D2_GPIO_PORT        GPIOD
#define AD7606_D2_GPIO_CLK         RCC_AHB1Periph_GPIOD
#define AD7606_D2_GPIO_PIN         GPIO_Pin_0
#define AD7606_D2_GPIO_PinSource   GPIO_PinSource0

#define AD7606_D3_GPIO_PORT        GPIOD
#define AD7606_D3_GPIO_CLK         RCC_AHB1Periph_GPIOD
#define AD7606_D3_GPIO_PIN         GPIO_Pin_1
#define AD7606_D3_GPIO_PinSource   GPIO_PinSource1

#define AD7606_D4_GPIO_PORT        GPIOE
#define AD7606_D4_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define AD7606_D4_GPIO_PIN         GPIO_Pin_7
#define AD7606_D4_GPIO_PinSource   GPIO_PinSource7

#define AD7606_D5_GPIO_PORT        GPIOE
#define AD7606_D5_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define AD7606_D5_GPIO_PIN         GPIO_Pin_8
#define AD7606_D5_GPIO_PinSource   GPIO_PinSource8

#define AD7606_D6_GPIO_PORT        GPIOE
#define AD7606_D6_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define AD7606_D6_GPIO_PIN         GPIO_Pin_9
#define AD7606_D6_GPIO_PinSource   GPIO_PinSource9

#define AD7606_D7_GPIO_PORT        GPIOE
#define AD7606_D7_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define AD7606_D7_GPIO_PIN         GPIO_Pin_10
#define AD7606_D7_GPIO_PinSource   GPIO_PinSource10

#define AD7606_D8_GPIO_PORT        GPIOE
#define AD7606_D8_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define AD7606_D8_GPIO_PIN         GPIO_Pin_11
#define AD7606_D8_GPIO_PinSource   GPIO_PinSource11

#define AD7606_D9_GPIO_PORT        GPIOE
#define AD7606_D9_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define AD7606_D9_GPIO_PIN         GPIO_Pin_12
#define AD7606_D9_GPIO_PinSource   GPIO_PinSource12

#define AD7606_D10_GPIO_PORT        GPIOE
#define AD7606_D10_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define AD7606_D10_GPIO_PIN         GPIO_Pin_13
#define AD7606_D10_GPIO_PinSource   GPIO_PinSource13

#define AD7606_D11_GPIO_PORT        GPIOE
#define AD7606_D11_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define AD7606_D11_GPIO_PIN         GPIO_Pin_14
#define AD7606_D11_GPIO_PinSource   GPIO_PinSource14

#define AD7606_D12_GPIO_PORT        GPIOE
#define AD7606_D12_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define AD7606_D12_GPIO_PIN         GPIO_Pin_15
#define AD7606_D12_GPIO_PinSource   GPIO_PinSource15

#define AD7606_D13_GPIO_PORT        GPIOD
#define AD7606_D13_GPIO_CLK         RCC_AHB1Periph_GPIOD
#define AD7606_D13_GPIO_PIN         GPIO_Pin_8
#define AD7606_D13_GPIO_PinSource   GPIO_PinSource8

#define AD7606_D14_GPIO_PORT        GPIOD
#define AD7606_D14_GPIO_CLK         RCC_AHB1Periph_GPIOD
#define AD7606_D14_GPIO_PIN         GPIO_Pin_9
#define AD7606_D14_GPIO_PinSource   GPIO_PinSource9

#define AD7606_D15_GPIO_PORT        GPIOD
#define AD7606_D15_GPIO_CLK         RCC_AHB1Periph_GPIOD
#define AD7606_D15_GPIO_PIN         GPIO_Pin_10
#define AD7606_D15_GPIO_PinSource   GPIO_PinSource10

// �����ź���  
// CSƬѡ 
// CS1 ,��Ӧ�Ļ���ַ0x60000000 
#define AD7606_CS1_GPIO_PORT        GPIOD
#define AD7606_CS1_GPIO_CLK         RCC_AHB1Periph_GPIOD
#define AD7606_CS1_GPIO_PIN         GPIO_Pin_7
#define AD7606_CS1_GPIO_PinSource   GPIO_PinSource7
// CS2 ,��Ӧ�Ļ���ַ0x64000000 
#define AD7606_CS2_GPIO_PORT        GPIOG
#define AD7606_CS2_GPIO_CLK         RCC_AHB1Periph_GPIOG
#define AD7606_CS2_GPIO_PIN         GPIO_Pin_9
#define AD7606_CS2_GPIO_PinSource   GPIO_PinSource9
// CS3 ,��Ӧ�Ļ���ַ0x68000000 
#define AD7606_CS3_GPIO_PORT        GPIOG
#define AD7606_CS3_GPIO_CLK         RCC_AHB1Periph_GPIOG
#define AD7606_CS3_GPIO_PIN         GPIO_Pin_10
#define AD7606_CS3_GPIO_PinSource   GPIO_PinSource10
// CS4 ,��Ӧ�Ļ���ַ0x6C000000 
#define AD7606_CS4_GPIO_PORT        GPIOG
#define AD7606_CS4_GPIO_CLK         RCC_AHB1Periph_GPIOG
#define AD7606_CS4_GPIO_PIN         GPIO_Pin_12
#define AD7606_CS4_GPIO_PinSource   GPIO_PinSource12

/*RD��ʹ��*/
#define AD7606_RD_GPIO_PORT         GPIOD
#define AD7606_RD_GPIO_CLK          RCC_AHB1Periph_GPIOD
#define AD7606_RD_GPIO_PIN          GPIO_Pin_4
#define AD7606_RD_GPIO_PinSource    GPIO_PinSource4

/*RANGE�ź�*/
#define AD7606_RANGE_GPIO_PORT      GPIOB
#define AD7606_RANGE_GPIO_CLK       RCC_AHB1Periph_GPIOB
#define AD7606_RANGE_GPIO_PIN       GPIO_Pin_8

/*RST��λ�ź�*/
#define AD7606_RST_GPIO_PORT        GPIOB
#define AD7606_RST_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define AD7606_RST_GPIO_PIN         GPIO_Pin_9

/*����������ѡ��*/
//OS0
#define AD7606_OS2_GPIO_PORT        GPIOA
#define AD7606_OS2_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define AD7606_OS2_GPIO_PIN         GPIO_Pin_6
//OS1
#define AD7606_OS1_GPIO_PORT        GPIOA
#define AD7606_OS1_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define AD7606_OS1_GPIO_PIN         GPIO_Pin_5
//OS2
#define AD7606_OS0_GPIO_PORT        GPIOA
#define AD7606_OS0_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define AD7606_OS0_GPIO_PIN         GPIO_Pin_4

//BUSYת����ʼ/����ź�(�����ж�) 
//BUSY1
#define AD7606_BUSY1_GPIO_PORT                GPIOB
#define AD7606_BUSY1_GPIO_CLK                 RCC_AHB1Periph_GPIOB
#define AD7606_BUSY1_GPIO_PIN                 GPIO_Pin_0
#define AD7606_BUSY1_EXTI_PortSource          EXTI_PortSourceGPIOB
#define AD7606_BUSY1_EXTI_PinSource           EXTI_PinSource0
#define AD7606_BUSY1_EXTI_LINE                EXTI_Line0
#define AD7606_BUSY1_EXTI_IRQ                 EXTI0_IRQn

#define BUSY1_IRQHandler                      EXTI0_IRQHandler

//BUSY2
#define AD7606_BUSY2_GPIO_PORT                GPIOB
#define AD7606_BUSY2_GPIO_CLK                 RCC_AHB1Periph_GPIOB
#define AD7606_BUSY2_GPIO_PIN                 GPIO_Pin_1
#define AD7606_BUSY2_EXTI_PortSource          EXTI_PortSourceGPIOB
#define AD7606_BUSY2_EXTI_PinSource           EXTI_PinSource1
#define AD7606_BUSY2_EXTI_LINE                EXTI_Line1
#define AD7606_BUSY2_EXTI_IRQ                 EXTI1_IRQn

#define BUSY2_IRQHandler                      EXTI1_IRQHandler

//BUSY3
#define AD7606_BUSY3_GPIO_PORT                GPIOC
#define AD7606_BUSY3_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define AD7606_BUSY3_GPIO_PIN                 GPIO_Pin_2
#define AD7606_BUSY3_EXTI_PortSource          EXTI_PortSourceGPIOC
#define AD7606_BUSY3_EXTI_PinSource           EXTI_PinSource2
#define AD7606_BUSY3_EXTI_LINE                EXTI_Line2
#define AD7606_BUSY3_EXTI_IRQ                 EXTI2_IRQn

#define BUSY3_IRQHandler                      EXTI2_IRQHandler

//BUSY4
#define AD7606_BUSY4_GPIO_PORT                GPIOC
#define AD7606_BUSY4_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define AD7606_BUSY4_GPIO_PIN                 GPIO_Pin_3
#define AD7606_BUSY4_EXTI_PortSource          EXTI_PortSourceGPIOC
#define AD7606_BUSY4_EXTI_PinSource           EXTI_PinSource3
#define AD7606_BUSY4_EXTI_LINE                EXTI_Line3
#define AD7606_BUSY4_EXTI_IRQ                 EXTI3_IRQn

#define BUSY4_IRQHandler                      EXTI3_IRQHandler

/* ֱ�Ӳ����Ĵ����ķ�������IO  
#define	digitalHi(p,i)			 p->BSRRL=i		//����ߵ�ƽ
#define digitalLo(p,i)			 p->BSRRH=i		//����͵�ƽ
#define digitalToggle(p,i)	 	 p->ODR ^=i		//�����ת״̬
*/
// ���ù�������
#define OS0_H()		 digitalHi(AD7606_OS0_GPIO_PORT,AD7606_OS0_GPIO_PIN)
#define OS0_L()		 digitalLo(AD7606_OS0_GPIO_PORT,AD7606_OS0_GPIO_PIN)
#define OS1_H()		 digitalHi(AD7606_OS1_GPIO_PORT,AD7606_OS1_GPIO_PIN)
#define OS1_L()		 digitalLo(AD7606_OS1_GPIO_PORT,AD7606_OS1_GPIO_PIN)
#define OS2_H()		 digitalHi(AD7606_OS2_GPIO_PORT,AD7606_OS2_GPIO_PIN)
#define OS2_L()      digitalLo(AD7606_OS2_GPIO_PORT,AD7606_OS2_GPIO_PIN)

#define AD_OS_NO    {OS2_L(); OS1_L(); OS0_L();} 
#define AD_OS_X2    {OS2_L(); OS1_L(); OS0_H();}  
#define AD_OS_X4    {OS2_L(); OS1_H(); OS0_L();} 
#define AD_OS_X8    {OS2_L(); OS1_H(); OS0_H();}  
#define AD_OS_X16   {OS2_H(); OS1_L(); OS0_L();}  
#define AD_OS_X32   {OS2_H(); OS1_L(); OS0_H();} 
#define AD_OS_X64   {OS2_H(); OS1_H(); OS0_L();}  

// ������Χѡ�� 
#define RANGE_10V()	 digitalHi(AD7606_RANGE_GPIO_PORT,AD7606_RANGE_GPIO_PIN)  //�ߵ�ƽ
#define RANGE_5V()	 digitalLo(AD7606_RANGE_GPIO_PORT,AD7606_RANGE_GPIO_PIN)  //�͵�ƽ
// ��λ
#define RST_H()		 digitalHi(AD7606_RST_GPIO_PORT,AD7606_RST_GPIO_PIN)
#define RST_L()		 digitalLo(AD7606_RST_GPIO_PORT,AD7606_RST_GPIO_PIN)
// ��IO��״̬
#define IS_BUSY1(x)  ((x)==GPIO_ReadInputDataBit(AD7606_BUSY1_GPIO_PORT,AD7606_BUSY1_GPIO_PIN)) 
#define IS_BUSY2(x)  ((x)==GPIO_ReadInputDataBit(AD7606_BUSY2_GPIO_PORT,AD7606_BUSY2_GPIO_PIN)) 
#define IS_BUSY3(x)  ((x)==GPIO_ReadInputDataBit(AD7606_BUSY3_GPIO_PORT,AD7606_BUSY3_GPIO_PIN)) 
#define IS_BUSY4(x)  ((x)==GPIO_ReadInputDataBit(AD7606_BUSY4_GPIO_PORT,AD7606_BUSY4_GPIO_PIN)) 

//4ƬAD7606�ĵ�ַ
#define AD7606_1_RESULT()	*(__IO uint16_t *)0x60000000 
#define AD7606_2_RESULT()	*(__IO uint16_t *)0x64000000 
#define AD7606_3_RESULT()	*(__IO uint16_t *)0x68000000 
#define AD7606_4_RESULT()	*(__IO uint16_t *)0x6C000000 


extern uint8_t AD_Run;             //AD����״̬0=ֹͣ��1=����
/*******************************************/
void AD7606_Init(void);
void AD7606_Reset(void);
void AD7606_StartConvst(uint8_t flag);
void AD7606_Stop(void);


											  


#endif

