/*******************************************************************************
*                     ��Ȩ���� (C), 2017-,NCUROBOT
********************************************************************************
* �� �� ��   : Motor_USE_CAN.c
* �� �� ��   : ����
* ��    ��   : NCURM
* ��������   : 2018��7��
* ����޸�   :
* ��������   : �����ģ����ʹ��CAN���п��Ƶĵ��
* �����б�   :

*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Motor_USE_CAN.h"
#include "protocol.h"
#include "SystemState.h"

/* �ڲ��Զ����������� --------------------------------------------------------*/

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/
/*******************Ħ���ֵ���͵��̵���Ĳ�������***************************/
moto_measure_t   moto_chassis_get[4] = {0};//4 �� 3508
moto_measure_t   moto_dial_get = {0};  //c2006
moto_measure_t   moto_moca_get[2] = {0};//2��Ħ���� 3508
moto_measure_t   pit_get;
moto_measure_t   yaw_get;
/* �ⲿ����ԭ������ ----------------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/
static CAN_TxHeaderTypeDef  Cloud_Platform_Data;
static CAN_TxHeaderTypeDef	 Chassis_Motor_Data;
static CAN_TxHeaderTypeDef  Allocate_Motor_Data;
static CAN_TxHeaderTypeDef  Moca_Motor_Data;
static CAN_TxHeaderTypeDef  CANSend_Remote;
static CAN_TxHeaderTypeDef  CANSend_Gimbal;
static CAN_TxHeaderTypeDef  CANSend_Error;
static CAN_TxHeaderTypeDef  CANSend_Cilent;

extern uint8_t CAN_RX_date[8];
/* ����ԭ������ ----------------------------------------------------------*/

/**
	**************************************************************
	** Descriptions: ��̨�����������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**					yaw:yaw�����ֵ
	**				pitch:pitch����ֵ
	** Output: NULL
	**************************************************************
**/
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
{
	  uint8_t CAN_TX_DATA[8];
	
		Cloud_Platform_Data.StdId = 0x1FF;
		Cloud_Platform_Data.IDE = CAN_ID_STD;
		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
		Cloud_Platform_Data.DLC = 0X08;
		
		CAN_TX_DATA[0] = yaw >> 8;
		CAN_TX_DATA[1] = yaw;
		CAN_TX_DATA[2] = pitch >> 8;
		CAN_TX_DATA[3] = pitch;
		CAN_TX_DATA[4] = 0x00;
		CAN_TX_DATA[5] = 0x00;
		CAN_TX_DATA[6] = 0x00;
		CAN_TX_DATA[7] = 0x00;

  	HAL_CAN_AddTxMessage(hcan, &Cloud_Platform_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}

/**
	**************************************************************
	** Descriptions: ��̨���У׼����
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**					
	**				
	** Output: NULL
	**************************************************************
**/
void Cloud_Platform_Motor_jiaozhun(CAN_HandleTypeDef * hcan)
{
	  uint8_t CAN_TX_DATA[8];
	
		Cloud_Platform_Data.StdId = 0x3F0;
		Cloud_Platform_Data.IDE = CAN_ID_STD;
		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
		Cloud_Platform_Data.DLC = 0X08;
		
		CAN_TX_DATA[0] = 'c' ;
		CAN_TX_DATA[1] = 0x00;
		CAN_TX_DATA[2] = 0x00;
		CAN_TX_DATA[3] = 0x00;
		CAN_TX_DATA[4] = 0x00;
		CAN_TX_DATA[5] = 0x00;
		CAN_TX_DATA[6] = 0x00;
		CAN_TX_DATA[7] = 0x00;

  	HAL_CAN_AddTxMessage(hcan, &Cloud_Platform_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}

/**
	**************************************************************
	** Descriptions: ��̨���ʧ�ܺ���
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**					
	**				
	** Output: NULL
	**************************************************************
**/
void Cloud_Platform_Motor_Disable(CAN_HandleTypeDef * hcan)
{
	  uint8_t CAN_TX_DATA[8];
	
		Cloud_Platform_Data.StdId = 0x1FF;
		Cloud_Platform_Data.IDE = CAN_ID_STD;
		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
		Cloud_Platform_Data.DLC = 0X08;
		
		CAN_TX_DATA[0] = 0x00;
		CAN_TX_DATA[1] = 0x00;
		CAN_TX_DATA[2] = 0x00;
		CAN_TX_DATA[3] = 0x00;
		CAN_TX_DATA[4] = 0x00;
		CAN_TX_DATA[5] = 0x00;
		CAN_TX_DATA[6] = 0x00;
		CAN_TX_DATA[7] = 0x00;

  	HAL_CAN_AddTxMessage(hcan, &Cloud_Platform_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}

/**
	**************************************************************
	** Descriptions: ���̵����������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN2
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	    uint8_t CAN_TX_DATA[8];
	
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			CAN_TX_DATA[0] = iq1 >> 8;
			CAN_TX_DATA[1] = iq1;
			CAN_TX_DATA[2] = iq2 >> 8;
			CAN_TX_DATA[3] = iq2;
			CAN_TX_DATA[4] = iq3 >> 8;
			CAN_TX_DATA[5] = iq3;
			CAN_TX_DATA[6] = iq4 >> 8;
			CAN_TX_DATA[7] = iq4;
	
			HAL_CAN_AddTxMessage(hcan, &Chassis_Motor_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	

/**
	**************************************************************
	** Descriptions: ���̵��ʧ�ܺ���
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN2
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor_Disable( CAN_HandleTypeDef * hcan)
{
	    uint8_t CAN_TX_DATA[8];
	
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			CAN_TX_DATA[0] = 0x00;
			CAN_TX_DATA[1] = 0x00;
			CAN_TX_DATA[2] = 0x00;
			CAN_TX_DATA[3] = 0x00;
			CAN_TX_DATA[4] = 0x00;
			CAN_TX_DATA[5] = 0x00;
			CAN_TX_DATA[6] = 0x00;
			CAN_TX_DATA[7] = 0x00;
	
			HAL_CAN_AddTxMessage(hcan, &Chassis_Motor_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	
/**
	**************************************************************
	** Descriptions: ���������������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**				value:��������ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value)
{
      uint8_t CAN_TX_DATA[8];
	
			Allocate_Motor_Data.DLC = 0x08;
			Allocate_Motor_Data.IDE = CAN_ID_STD;
			Allocate_Motor_Data.RTR = CAN_RTR_DATA;
			Allocate_Motor_Data.StdId = 0x200;

			CAN_TX_DATA[0] = value >> 8;
			CAN_TX_DATA[1] = value;
			CAN_TX_DATA[2] = 0;
			CAN_TX_DATA[3] = 0;
			CAN_TX_DATA[4] = 0;
			CAN_TX_DATA[5] = 0;
			CAN_TX_DATA[6] = 0;
			CAN_TX_DATA[7] = 0;
	
			HAL_CAN_AddTxMessage(hcan, &Allocate_Motor_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}
/**
	**************************************************************
	** Descriptions: Ħ���ֵ��3508��������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**				value:��������ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Moca_Motor(CAN_HandleTypeDef * hcan,int16_t wheel_left, int16_t wheel_right)
{
      uint8_t CAN_TX_DATA[8];
			Moca_Motor_Data.DLC = 0x08;
			Moca_Motor_Data.IDE = CAN_ID_STD;
			Moca_Motor_Data.RTR = CAN_RTR_DATA;
			Moca_Motor_Data.StdId = 0x200;//

			CAN_TX_DATA[0] = 0;
			CAN_TX_DATA[1] = 0;
			CAN_TX_DATA[2] = wheel_left >> 8;  
			CAN_TX_DATA[3] = wheel_left;
			CAN_TX_DATA[4] = wheel_right >> 8;
			CAN_TX_DATA[5] = wheel_right;
			CAN_TX_DATA[6] = 0;
			CAN_TX_DATA[7] = 0;
	
			HAL_CAN_AddTxMessage(hcan, &Moca_Motor_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}

/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��6623����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_6623(moto_measure_t *ptr,uint8_t CAN_RX_date[])
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->real_current  = (int16_t)(CAN_RX_date[2]<<8 | CAN_RX_date[3]);
	ptr->given_current = (int16_t)(CAN_RX_date[4]<<8 | CAN_RX_date[5]);
	ptr->speed_rpm = ptr->real_current;
//	ptr->hall = hcan->pRxMsg->Data[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��3508����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_3508(moto_measure_t *ptr,uint8_t CAN_RX_date[])
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->speed_rpm  = (int16_t)(CAN_RX_date[2]<<8 | CAN_RX_date[3]);
	ptr->real_current = (int16_t)(CAN_RX_date[4]<<8 | CAN_RX_date[5]);
	ptr->hall = CAN_RX_date[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��6020����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_6020(moto_measure_t *ptr,uint8_t CAN_RX_date[])
{
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->speed_rpm  = (int16_t)(CAN_RX_date[2]<<8 | CAN_RX_date[3]);
	ptr->given_current = (int16_t)(CAN_RX_date[4]<<8 | CAN_RX_date[5]);

	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
/**
	**************************************************************
	** Descriptions:��ȡ�������ֵ��ƫ��ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr,uint8_t CAN_RX_date[])
{
//	ptr->angle = 7200 ;
//	ptr->offset_angle = 7200;
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->offset_angle = ptr->angle;
}




/**
	**************************************************************
	** Descriptions: ����ͨ�ţ����͸����̵�ң�������ݣ�
	** Input: 	ң��������
	**			  
	**				
	** Output: NULL
	**************************************************************
**/
void CAN_Send_Remote( CAN_HandleTypeDef * hcan,
									                int16_t key_v, int16_t rc_ch0, int16_t rc_ch1, uint8_t rc_s1, uint8_t rc_s2)
{
	    uint8_t CAN_TX_DATA[8];
	
			CANSend_Remote.DLC = 0x08;
			CANSend_Remote.IDE = CAN_ID_STD;
			CANSend_Remote.RTR = CAN_RTR_DATA;
			CANSend_Remote.StdId = 0x110;

			CAN_TX_DATA[0] = key_v >> 8;
			CAN_TX_DATA[1] = key_v;
			CAN_TX_DATA[2] = rc_ch0 >> 8;
			CAN_TX_DATA[3] = rc_ch0;
			CAN_TX_DATA[4] = rc_ch1 >> 8;
			CAN_TX_DATA[5] = rc_ch1;
			CAN_TX_DATA[6] = rc_s1;
			CAN_TX_DATA[7] = rc_s2;
	
			HAL_CAN_AddTxMessage(hcan, &CANSend_Remote, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	


/**
	**************************************************************
	** Descriptions: ����ͨ�ţ����͸����̵���̨��Ϣ��
	** Input: 	��̨��Ϣ
	**			  
	**				
	** Output: NULL
	**************************************************************
**/
void CAN_Send_Gimbal( CAN_HandleTypeDef * hcan, moto_measure_t * yaw_get, Gimbal_Status_t * gimbal_status)
{
	    uint8_t CAN_TX_DATA[8];
	
			CANSend_Gimbal.DLC = 0x08;
			CANSend_Gimbal.IDE = CAN_ID_STD;
			CANSend_Gimbal.RTR = CAN_RTR_DATA;
			CANSend_Gimbal.StdId = 0x120;

			CAN_TX_DATA[0] = yaw_get->angle >> 8;
			CAN_TX_DATA[1] = yaw_get->angle;
			CAN_TX_DATA[2] = yaw_get->total_angle >> 8;
			CAN_TX_DATA[3] = yaw_get->total_angle;
			CAN_TX_DATA[4] = gimbal_status->minipc_mode;
			CAN_TX_DATA[5] = gimbal_status->gimbal_mode;
			CAN_TX_DATA[6] = gimbal_status->remote_mode;
			CAN_TX_DATA[7] = gimbal_status->gimbal_flag;
	
			HAL_CAN_AddTxMessage(hcan, &CANSend_Gimbal, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	



void CAN_Send_Error( CAN_HandleTypeDef * hcan, int16_t OutLine_Flag, int16_t task_OutLine_Flag )//����
{
	    uint8_t CAN_TX_DATA[8];
	
			CANSend_Error.DLC = 0x08;
			CANSend_Error.IDE = CAN_ID_STD;
			CANSend_Error.RTR = CAN_RTR_DATA;
			CANSend_Error.StdId = 0x119;

			CAN_TX_DATA[0] = OutLine_Flag >> 8;
			CAN_TX_DATA[1] = OutLine_Flag;
			CAN_TX_DATA[2] = task_OutLine_Flag >> 8;
			CAN_TX_DATA[3] = task_OutLine_Flag;
			CAN_TX_DATA[4] = 0;
			CAN_TX_DATA[5] = 0;
			CAN_TX_DATA[6] = 0;
			CAN_TX_DATA[7] = 0;
	
			HAL_CAN_AddTxMessage(hcan, &CANSend_Error, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	


void CAN_Send_cilent( CAN_HandleTypeDef * hcan, uint8_t flag0, uint8_t flag1 , uint8_t flag2 ,uint8_t flag3)//����
{
	    uint8_t CAN_TX_DATA[8];
	
			CANSend_Cilent.DLC = 0x08;
			CANSend_Cilent.IDE = CAN_ID_STD;
			CANSend_Cilent.RTR = CAN_RTR_DATA;
			CANSend_Cilent.StdId = 0x130;

			CAN_TX_DATA[0] = flag0;
			CAN_TX_DATA[1] = flag1;
			CAN_TX_DATA[2] = flag2;
			CAN_TX_DATA[3] = flag3;
			CAN_TX_DATA[4] = 0;
			CAN_TX_DATA[5] = 0;
			CAN_TX_DATA[6] = 0;
			CAN_TX_DATA[7] = 0;
	
			HAL_CAN_AddTxMessage(hcan, &CANSend_Cilent, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	

void CAN_GET_CP( CAN_HandleTypeDef * hcan)
{
//    	Robot.heat.shoot_17_cooling_limit = (hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	    Robot.heat.shoot_17_heat = (hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]) ;
//	    Robot.id =  hcan->pRxMsg->Data[4] ;

}	

void CAN_GET_ERROR( CAN_HandleTypeDef * hcan)
{
//    Robot.level = hcan->pRxMsg->Data[4];
//    Robot.Hp_ratio = hcan->pRxMsg->Data[5];
//    Robot.heat.shoot_17_cooling_rate = (hcan->pRxMsg->Data[6]<<8 | hcan->pRxMsg->Data[7]) ;
}
