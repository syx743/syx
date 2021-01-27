/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "communication.h"
#include "gimbal_control.h"
#include "SystemState.h"
#include "Motor_USE_CAN.h"
#include "tim.h"
#include "can.h"
#include "INS_task.h"
#include "ist8310driver.h"
#include "ist8310driver_middleware.h"
#include "protocol.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
CAN_RxHeaderTypeDef  CAN1_Rx_Header;
CAN_RxHeaderTypeDef  CAN2_Rx_Header;
uint8_t CAN1_RX_date[8];
uint8_t CAN2_RX_date[8];
uint32_t DMA_FLAGS;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern  osThreadId RemoteDataTaskHandle;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim12;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
//void SysTick_Handler(void)
//{
//  /* USER CODE BEGIN SysTick_IRQn 0 */

//  /* USER CODE END SysTick_IRQn 0 */
//  osSystickHandler();
//  /* USER CODE BEGIN SysTick_IRQn 1 */

//  /* USER CODE END SysTick_IRQn 1 */
//}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}


/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //�����Ƕ�ȡ���
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);    //���������ݵ�[2]λ ��1�������������
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);  //���������ݵ�[1]λ ��0

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read over
        //���ٶȼƶ�ȡ���
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //�¶ȶ�ȡ���
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))   //���������ݽ�����ɺ�ͨ������жϿ�������֪ͨ
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}


/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  static  BaseType_t  pxHigherPriorityTaskWoken;
	uint8_t tmp1,tmp2;
	tmp1 = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE);   //�����ж��н������ֽ���ȡ����ֹͣDMA
  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE);
  /* USER CODE END USART3_IRQn 0 */
	if((tmp1 != RESET) && (tmp2 != RESET))
  { 
   	__HAL_DMA_DISABLE(&hdma_usart3_rx);
		
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart3_rx);	
		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAGS);
		
		__HAL_DMA_SET_COUNTER(&hdma_usart3_rx,SizeofRemote);
		__HAL_DMA_ENABLE(&hdma_usart3_rx);
				
  /* USER CODE BEGIN UART8_IRQn 1 */
    vTaskNotifyGiveFromISR(RemoteDataTaskHandle,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);			
		
						/*���IDLE��־λ*/
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);	
	}
//  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  static  BaseType_t  pxHigherPriorityTaskWoken;
	uint8_t tmp1,tmp2;
	tmp1 = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE);   //�����ж��н������ֽ���ȡ����ֹͣDMA
  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE);
  /* USER CODE END USART3_IRQn 0 */
	if((tmp1 != RESET) && (tmp2 != RESET))
  { 
   	__HAL_DMA_DISABLE(&hdma_usart1_rx);
		
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_rx);	
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAGS);
		
		__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,SizeofRemote);
		__HAL_DMA_ENABLE(&hdma_usart1_rx);
				
  /* USER CODE BEGIN UART8_IRQn 1 */
    vTaskNotifyGiveFromISR(RemoteDataTaskHandle,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);			
		
						/*���IDLE��־λ*/
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);	
	}
//  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
//	 static  BaseType_t  pxHigherPriorityTaskWoken;
//	uint8_t tmp1,tmp2;
//	tmp1 = __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE);   //�����ж��н������ֽ���ȡ����ֹͣDMA
//  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE);
//  /* USER CODE END USART6_IRQn 0 */
//	if((tmp1 != RESET) && (tmp2 != RESET))
//  { 
//    
//	__HAL_DMA_DISABLE(&hdma_usart6_rx);
//		
//	DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart6_rx);	
//	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAGS);
//  Referee_Data_Handler();
//	__HAL_DMA_SET_COUNTER(&hdma_usart6_rx,SizeofMinipc);
//	__HAL_DMA_ENABLE(&hdma_usart6_rx);

//	vTaskNotifyGiveFromISR(MiniPCDataTaskHandle,&pxHigherPriorityTaskWoken);
//	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);		

//				/*���IDLE��־λ*/
//   __HAL_UART_CLEAR_IDLEFLAG(&huart6);		
//	}
//  HAL_UART_IRQHandler(&huart6);
	uint8_t tmp1,tmp2;
	tmp1 = __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE);   //�����ж��н������ֽ���ȡ����ֹͣDMA
  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE);
	
   if((tmp1 != RESET))
	{
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		
		//RefreshDeviceOutLineTime(JY61_NO);
		
		 USART6_RX_NUM=(SizeofReferee)-(hdma_usart6_rx.Instance->NDTR);
		 Referee_Data_Handler();
		
		__HAL_DMA_SET_COUNTER(&hdma_usart6_rx,SizeofReferee);
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
	}
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) 
	{
    HAL_IncTick();
  }else if(htim == (&htim6))
	{
		RefreshSysTime();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM1) 
	{
		__HAL_TIM_ENABLE(&htim1);
		__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);
  }
  /* USER CODE END Callback 1 */
}
uint8_t sy=0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_Rx_Header, CAN1_RX_date);
		switch(CAN1_Rx_Header.StdId)
		{
			case 0x206:                            //pitch��������
			{
				
				
				if(pit_get.msg_cnt++ <= 10)   //���˵���ʼ���յ����ݣ���ֹ��ʼ�������ݳ���
				{
					pit_get.angle = Pitch_Middle_Angle ;           //��������Ҫ        
          pit_get.offset_angle = Pitch_Middle_Angle;     //��׼λ��ѡ����ͨ��pitch���׼λ�õĵ���Ƕȷ�������
				}else
				{
					pit_get.msg_cnt = 11;
					get_moto_measure_6020(&pit_get,CAN1_RX_date);
				}
			}break;
			case 0x205:                            //yaw��������
			{
				sy=1;
				if(yaw_get.msg_cnt++ <= 10)
				{
          yaw_get.angle = Yaw_Middle_Angle ;            //��������Ҫ
          yaw_get.offset_angle = Yaw_Middle_Angle;      //��׼λ��ѡ����ͨ��yaw���׼λ�õĵ���Ƕȷ�������
				}else
				{
					yaw_get.msg_cnt = 11;
					get_moto_measure_6020(&yaw_get,CAN1_RX_date);
				}
			}break;
			case 0x201:
			{
								
				if(moto_dial_get.msg_cnt++ <= 10)	
				{
					get_moto_offset(&moto_dial_get,CAN1_RX_date);
				}
				else
				{	
					moto_dial_get.msg_cnt = 11;	
					get_moto_measure_6623(&moto_dial_get,CAN1_RX_date);
				}
			}break;
			default: break;
		}
	}else if(hcan == &hcan2)
	{
	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_Rx_Header, CAN2_RX_date);
		switch(CAN2_Rx_Header.StdId)
		{ case 0x110:                            //Ħ����3508��
			{
				uint8_t vxcxca=1;
				
			}break;
		  case 0x205:                            //yaw��������
			{
       RefreshDeviceOutLineTime(MotorY_NO);
				
				if(yaw_get.msg_cnt++ <= 10)
				{
          yaw_get.angle = Yaw_Middle_Angle ;            //��������Ҫ
          yaw_get.offset_angle = Yaw_Middle_Angle;      //��׼λ��ѡ����ͨ��yaw���׼λ�õĵ���Ƕȷ�������
				}else
				{
					yaw_get.msg_cnt = 11;
					get_moto_measure_6020(&yaw_get,CAN2_RX_date);
				}
			}break;
		 case 0x202:                            //Ħ����3508����
			{
//       RefreshDeviceOutLineTime(MotorY_NO);
				
					if(moto_moca_get[0].msg_cnt++ <= 50)	
					{
						 get_moto_offset(&moto_moca_get[0], CAN2_RX_date);
					}else
					{		
					moto_moca_get[0].msg_cnt = 51;	
					get_moto_measure_3508(&moto_moca_get[0],CAN2_RX_date);
//					RefreshDeviceOutLineTime(MotorY_NO);
				}
			}break;
		 case 0x203:                            //Ħ����3508��
			{
//       RefreshDeviceOutLineTime(MotorY_NO);
				
					if(moto_moca_get[1].msg_cnt++ <= 50)	
					{
						 get_moto_offset(&moto_moca_get[1], CAN2_RX_date);
					}else
					{		
					moto_moca_get[1].msg_cnt = 51;	
					get_moto_measure_3508(&moto_moca_get[1],CAN2_RX_date);
//					RefreshDeviceOutLineTime(MotorY_NO);
				}
			}break;
      case 0x160  :
      {
        CAN_GET_ERROR(&hcan2);
      }
      break;
      
      case 0x150  :
      {
        CAN_GET_CP(&hcan2);
      }
      break;
      default : break;
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)    //�ж���һ֡���ٶȼ������Ƿ������
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;             //���ٶ����ݱ�־λ��[0]λ ��1
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;        //�¶����ݱ�־λ��[0]λ ��1�������ɽ�����һ֡���ݽ���
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)  //�ж���һ֡�����������Ƿ������
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS; //���������ݱ�־λ��[0]λ ��1�������ɽ�����һ֡���ݽ���
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == IST8310_DRDY_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;

        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1<< IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);

            ist8310_read_mag(ist8310_real_data.mag);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        //wake up the task
        //��������
////        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
////        {
//            static BaseType_t xHigherPriorityTaskWoken;
//            vTaskNotifyGiveFromISR(imuTaskHandle, &xHigherPriorityTaskWoken);
//            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
////        }
    }


}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
