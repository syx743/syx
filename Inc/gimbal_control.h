#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "pid.h"
#include "communication.h "
#include "Motor_USE_CAN.h"
//#include "chassis_control.h"
#include "atom_imu.h"
#include "decode.h"

/* ��ģ�����ⲿ�ṩ�ĺ궨��--------------------------------------------------*/	 
#define GIMBAL_PERIOD 1
#define Yaw_Middle_Angle 7462
#define Pitch_Middle_Angle 7000
/* ��ģ�����ⲿ�ṩ���������Ͷ���--------------------------------------------*/
typedef struct{
		int32_t expect;           //����pid����ֵ
		uint8_t	step;
		uint8_t mode;
		int32_t expect_pc;        //�Ӿ��趨ֵ
    int16_t err;
} Pos_Set;


/* ��ģ�����ⲿ�ṩ�ĺ궨��--------------------------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľӿڳ�������--------------------------------------------*/

extern Pos_Set  yaw_tly_set;
extern Pos_Set  pit_set;
extern Pos_Set  yaw_set;
extern int8_t gimbal_disable_flg;
extern Gimbal_Status_t gimbal_status;
extern int32_t pitch_zhi ;
extern int32_t yaw_zhi ;
extern pid_t pid_pit_start ;
extern pid_t pid_pit_start_spd ;
extern pid_t pid_yaw_start ;
extern pid_t pid_yaw_start_spd ;
/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������----------------------------------------*/
void Gimbal_Task(void const * argument);
void Gimbal_angle_Conversion(moto_measure_t *ptr);

/* ȫ��������----------------------------------------------------------------*/

#endif
