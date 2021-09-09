/*****************************************************************************
    bsp_IO.c
ָʾ�ơ����������������õ�IO��ʼ��
******************************************************************************/

#include "bsp_io.h"   

/**********************************************************
    IO_GPIO_Config
���������õ�GPIO��ʼ��
**********************************************************/
void IO_GPIO_Config(void)
{       
    /*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /*������ص�GPIO����ʱ��*/
    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOF, ENABLE); 
    //�����
    /*��������ģʽΪ���ģʽ*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
    /*�������ŵ��������Ϊ�������*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
    /*��������Ϊ����ģʽ*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    /*������������Ϊ50MHz */   
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

    //�����
    // ��������Ϊ����ģʽ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
    // ������������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

    //Key1~3
    GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
    GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);   

    GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
    GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);   

    GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
    GPIO_Init(KEY3_GPIO_PORT, &GPIO_InitStructure);   

    //IO�ڳ�ʼ̬
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();       
    LED4_OFF();
    LED5_OFF();
    BELL_OFF();
}
/*********************************************END OF FILE**********************/
