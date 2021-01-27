/*******************************************************************************
                      ��Ȩ���� (C), 2017-, Mushiny
 *******************************************************************************
  �� �� ��   : communication.c
  �� �� ��   : ����
  ��    ��   : liyifeng
  ��������   : 2018��7��
  ����޸�   :
  ��������   : ����ͨ�ſ�
  �����б�   :
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "communication.h "
#include "Power_restriction.h"


/* �ڲ��Զ����������� --------------------------------------------------------*/
 uint8_t USART3_RX_DATA[(SizeofRemote)];//ң��
 uint16_t USART3_RX_NUM;
//uint8_t USART6_RX_DATA[(SizeofReferee)];//����ϵͳ
// uint16_t USART6_RX_NUM;
  uint8_t USART1_RX_DATA[(SizeofRemote)];//ң��
 uint16_t USART1_RX_NUM;
  uint8_t USART6_RX_DATA[(SizeofRemote)];//ң��
 uint16_t USART6_RX_NUM;


/* �ڲ��궨�� ----------------------------------------------------------------*/

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/

/* �ⲿ����ԭ������ ----------------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

//mpu6500
uint8_t MPU_id = 0;
int16_t gy_data_filter[5];
int16_t gz_data_filter[5];
RC_Ctl_t RC_Ctl; //ң������
IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

/* �ڲ�����ԭ������ ----------------------------------------------------------*/

/* �������岿�� --------------------------------------------------------------*/



/***************************************************************************************
**
	*	@brief	Remote_Ctrl(void)
	*	@param
	*	@supplement	ʹ�ô��ڽ���ң��������
	*	@retval	
****************************************************************************************/
void Remote_Ctrl(void)//ң�����ݽ���
{ 
	 uint8_t  * buff = USART3_RX_DATA;
	 RC_Ctl.rc.ch0 = (buff[0]| (buff[1] << 8)) & 0x07ff; //!< Channel 0 
	 RC_Ctl.rc.ch1 = ((buff[1] >> 3) | (buff[2] << 5)) & 0x07ff; //!< Channel 1 
	 RC_Ctl.rc.ch2 = ((buff[2] >> 6) | (buff[3] << 2) | //!< Channel 2 
	 (buff[4] << 10)) & 0x07ff; 
	 RC_Ctl.rc.ch3 = ((buff[4] >> 1) | (buff[5] << 7)) & 0x07ff; //!< Channel 3 
	 RC_Ctl.rc.s1 = ((buff[5] >> 4)& 0x000C) >> 2; //!< Switch left 
	 RC_Ctl.rc.s2 = ((buff[5] >> 4)& 0x0003); //!< Switch right 
	 RC_Ctl.mouse.x = buff[6] | (buff[7] << 8); //!< Mouse X axis 
	 RC_Ctl.mouse.y = buff[8] | (buff[9] << 8); //!< Mouse Y axis 
	 RC_Ctl.mouse.z = buff[10] | (buff[11] << 8); //!< Mouse Z axis 
	 RC_Ctl.mouse.press_l = buff[12]; //!< Mouse Left Is Press ? 
	 RC_Ctl.mouse.press_r = buff[13]; //!< Mouse Right Is Press ? 
	 RC_Ctl.key.v = buff[14] | (buff[15] << 8); //!< KeyBoard value 
}

/***************************************************************************************
**
	*	@brief	Remote_Ctrl(void)
	*	@param
	*	@supplement	ʹ�ô��ڽ���ң��������
	*	@retval	
****************************************************************************************/
void Remote_Disable(void)//ң�����ݽ���
{ 

	 RC_Ctl.rc.ch0 = 0; //!< Channel 0 
	 RC_Ctl.rc.ch1 = 0; //!< Channel 1 
	 RC_Ctl.rc.ch2 = 0; 
	 RC_Ctl.rc.ch3 = 0; //!< Channel 3 
	 RC_Ctl.rc.s1 = 0; //!< Switch left 
	 RC_Ctl.rc.s2 = 0; //!< Switch right 
	 RC_Ctl.mouse.x = 0; //!< Mouse X axis 
	 RC_Ctl.mouse.y = 0; //!< Mouse Y axis 
	 RC_Ctl.mouse.z = 0; //!< Mouse Z axis 
	 RC_Ctl.mouse.press_l = 0; //!< Mouse Left Is Press ? 
	 RC_Ctl.mouse.press_r = 0; //!< Mouse Right Is Press ? 
	 RC_Ctl.key.v = 0; //!< KeyBoard value 
}







