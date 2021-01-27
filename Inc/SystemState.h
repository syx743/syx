#ifndef __SysState_H__
#define __SysState_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"


#define OutLine_Time 100 //���߼��ʱ��
#define ShangDan_TIME  10000  //�ϵ�ʱ��


#define MyFlagSet(x,y) x=x|(0x00000001<<y) //���ñ�־λ  y�ڼ�λ
#define MyFlagClear(x,y) x=x&~(0x00000001<<y)
#define MyFlagGet(x,y) (x&(0x00000001<<y))

typedef struct
{
	short Mode;//����ģʽ
	short Enable;//״̬
	short State;//״̬
	short Task;//����
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//ʱ����������
	uint16_t OutLine_Flag;       //���߱�־
	uint16_t task_OutLine_Flag;  //������߱�־	
//	RobotDistDef RobotDist;//�����˲���
}SystemStateDef;

typedef enum
{
		Remote_NO,
		MotorY_NO,
		MotorP_NO,
  	MotorB_NO,
	  Minipc_NO,
	  WeiDong_NO,
  
		DeviceTotal_No	
}DeviceX_NoDEF;

typedef enum
{
	testTask_ON,
	RemoteDataTask_ON,
	GimbalContrlTask_ON,
	GunTask_ON,
	LedTask_ON,
	vOutLineCheckTask_ON,
	
	TASKTotal_No	
}TASK_NoDEF;

extern SystemStateDef SystemState;

int SystemState_Inite(void);//SystemState��ʼ��
void RefreshSysTime(void);//ˢ��ϵͳʱ�䣨mm��
float GetSystemTimer(void);//��ȡϵͳ��ǰ׼ȷʱ��


void OutLine_Check(void);//���߼����
void TASK_Check(void);//������
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//ˢ������ͨ��ʱ��ʱ������
void RefreshTaskOutLineTime(TASK_NoDEF Task_No);
void limit_check(void);

void vOutLineCheck_Task(void const *argument);





#endif
