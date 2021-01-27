#include "BSP.h"

void BSP_Init(void)
{
	MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
	delay_init();
	
	/*CAN������*/
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	
		/*Ħ����*/
//	GUN_Init();
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	/*ʹ�ܴ��ڵ�DMA���գ��������ڿ����ж�*/
	Bsp_UART_Receive_IT(&huart1,USART1_RX_DATA,SizeofRemote); 
	Bsp_UART_Receive_IT(&huart6,USART6_RX_DATA,SizeofRemote); //��һ����Ŀ���Ǵ���һ�ν����ڴ�
//	Bsp_UART_Receive_IT(&huart6,USART6_RX_DATA,SizeofReferee);
	/*ʹ��can�ж�*/
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}

