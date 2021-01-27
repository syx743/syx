/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gimbal_control.h"
#include "data_processing.h"
#include "SystemState.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId GimbalTaskHandle;	
osThreadId RemoteDataTaskHandle; 
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	osThreadDef(GimbalTask, Gimbal_Task, osPriorityNormal, 0, 256);                 //云台控制
	GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

	osThreadDef(RemoteDataTask, Remote_Data_Task, osPriorityHigh, 0, 256);          //遥控器数据处理
	RemoteDataTaskHandle = osThreadCreate(osThread(RemoteDataTask), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Remote_Data_Task(void const * argument)
{
	  uint32_t NotifyValue;

		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //未有任务通知则进入堵塞状态去等待任务通知
		
    if(NotifyValue == 1)
		{
			RefreshTaskOutLineTime(RemoteDataTask_ON);
			
			Remote_Ctrl();   //接收机原始数据处理
			
//			Send_MiniPC_Data(gimbal_status.minipc_color,gimbal_status.minipc_mode,yaw_get.offset_angle,pit_get.offset_angle,0);
//			CAN_Send_Remote(&hcan2,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
			
			switch(RC_Ctl.rc.s1)  //左拨钮
			{
					case 1: //上
					{
						gimbal_status.remote_mode = 0;
						MouseKeyControlProcess();
					}break; 
					case 2: //下
					{
						gimbal_status.remote_mode = 0;
				    RemoteControlProcess();
					}break; 
					case 3: //中
					{
						gimbal_status.remote_mode = 1;
					}break;  
					default :break;
			}
		
		}
		
		osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}     
/* USER CODE END Application */
void MouseKeyControlProcess()
{
	
//		if(F_Press){
//			test=1;
//			gimbal_status.gimbal_mode = 1;  
//		}
//		if(CTRL_Press && F_Press){
//			test=2;
//			gimbal_status.gimbal_mode = 2;  
//		}
//		
//		
//		if(R_Press){
//			gimbal_status.gimbal_mode = 3;
//		}
//		if(CTRL_Press&&R_Press){
//			gimbal_status.gimbal_mode = 2;
//		}
//		if(gimbal_status.bopan_mode!=bopan_Lianfa_mode){
//			gimbal_status.bopan_mode=bopan_Stop;
//		}
//		if(Left_Press){
//			gimbal_status.bopan_mode = bopan_danfa_mode;
//		}
//		if(Right_Press){
//			gimbal_status.bopan_mode=bopan_Lianfa_mode;
//		}
//		
//		if(C_Press){
////			Mocha_flag=1;                                                                        
//		}
//		if(C_Press&&CTRL_Press){
////			Mocha_flag=0;
//		}
//			
//		
//			
//			
//		
//		
//    
//	
//	if((gimbal_status.gimbal_mode == 1) || (gimbal_status.gimbal_mode == 3))            //陀螺仪模式
//		{
//			  yaw_tly_set.expect = yaw_tly_set.expect + RC_Ctl.mouse.x*1.2f;	
//				pit_set.expect = pit_set.expect + RC_Ctl.mouse.y *1.2f;	              //pit_set.expect_remote不受模式影响		
//			
//		}else if(gimbal_status.gimbal_mode == 2 )       // 编码器模式  
//		{
//				yaw_set.expect = yaw_set.expect - RC_Ctl.mouse.x*1.0f;
//		  	pit_set.expect = pit_set.expect + RC_Ctl.mouse.y *1.0f;	  
//		}
////		if(pit_set.expect<6200) pit_set.expect=6200;
////		else if(pit_set.expect>7480)  pit_set.expect=7480;

//             
		
}

void RemoteControlProcess()  
{
		switch(RC_Ctl.rc.s2)
		{
		
//				case 1: //上
//				{
//						gimbal_status.gimbal_mode = 2;  
//					yaw_set.expect =yaw_set.expect + (RC_Ctl.rc.ch2 - 0x400) / 10;
//  		  	pit_set.expect = pit_set.expect + (0x400 - RC_Ctl.rc.ch3) / 20;	
//				}break; 
//				case 2: //下
//				{
//						gimbal_status.gimbal_mode = 2;  
//					yaw_set.expect=0;
//				}break; 
//				case 3: //中
//				{ 
//						gimbal_status.gimbal_mode = 2;  
//					yaw_set.expect=1000;	 
//				}break; 
//				default :break;
//			}
				
			case 3: //上
			{
				 gimbal_status.gimbal_mode = 1;            //陀螺仪模式
//				 Set_AX6(2,819,0x3ff);       //逆时针角度   240° 关
//				gimbal_status.bopan_mode = bopan_Stop;
			}break; 
			case 2: //
			{
						gimbal_status.gimbal_mode = 3;            //小陀螺模式
//				 Set_AX6(2,819,0x3ff);       //逆时针角度   240° 关
//				gimbal_status.bopan_mode = bopan_Lianfa_mode;
			   
			}break; 
			case 1: //
			{
						gimbal_status.gimbal_mode = 2;            //编码器模式  

//				 Set_AX6(2,512,0x3ff);       //逆时针角度   150°
//         gimbal_status.bopan_mode =bopan_Stop ;
				
//				gimbal_status.gimbal_mode = 4;    //视觉模式
//				gimbal_status.minipc_mode = 1;    //自瞄模式
			}break; 
			default :break;
		}					
        
    if((gimbal_status.gimbal_mode == 1) || (gimbal_status.gimbal_mode == 3))            //陀螺仪模式
		{
			  yaw_tly_set.expect = yaw_tly_set.expect + (RC_Ctl.rc.ch2 - 0x400) / 10;	
				pit_set.expect = pit_set.expect + (0x400 - RC_Ctl.rc.ch3) / 20;	              //pit_set.expect_remote不受模式影响		
			
		}else if(gimbal_status.gimbal_mode == 2 )       // 编码器模式  
		{
				yaw_set.expect = yaw_set.expect + (RC_Ctl.rc.ch2 - 0x400) / 10;
		  	pit_set.expect = pit_set.expect + (0x400 - RC_Ctl.rc.ch3) / 20;	 
		}

		
		if(pit_set.expect<-800) pit_set.expect=-800;
		else if(pit_set.expect>370)  pit_set.expect=370;


//				  if(RC_Ctl.rc.s2 == 1)            //上
//		{    
//			 gimbal_status.bopan_mode= bopan_danfa_mode;
//		}
//		else if(RC_Ctl.rc.s2== 2)       //下
//		{
//        gimbal_status.bopan_mode = bopan_Lianfa_mode;
//		}else                             //中
//		{
//        gimbal_status.bopan_mode = bopan_Stop;    
//		}		
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
