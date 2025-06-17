
	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pid�ĵ��ν��棬��h�����޸����Ӽ��ԣ���������������ϵͳ�Ĳ���
  * @brief   
  ******************************************************************************
  * @attention	
  * 
  * 
  *		����������ÿ��������ɵĿ���������ԽС���Խ�쵫�����������Խ��
  * 
  * 
  * 
	*
	* 
  ******************************************************************************
  */
#include "car_params_config.h"

//--------------------------Left-------------------------
//____________________�ٶȻ�__________________________
//����PID����
float Left_KP_SPEED = 2000;//5000;
float Left_KI_SPEED = 120;//200
float Left_KD_SPEED = 0;//10000
//�������
float Left_DeadZone_SPEED = 0;
//���ַ��룬�����޷�
float Left_Separation_Threshold_SPEED = 10;
float Left_Integral_Limit_Max_SPEED = 8400;
float Left_Integral_Limit_Min_SPEED = -8400;
//����ܵ����
float Left_OutPut_Max_SPEED = 8400;
float Left_OutPut_Min_SPEED = -8400;
//____________________λ�û�__________________________
//����PID����
float Left_KP_DISTANCE = 6;
float Left_KI_DISTANCE = 0;
float Left_KD_DISTANCE = 5;
//�������
float Left_DeadZone_DISTANCE = 0.05;
//���ַ��룬�����޷�
float Left_Separation_Threshold_DISTANCE = 100;
float Left_Integral_Limit_Max_DISTANCE = 10000000;
float Left_Integral_Limit_Min_DISTANCE = -10000000;
//����ܵ����
float Left_OutPut_Max_DISTANCE = 1;
float Left_OutPut_Min_DISTANCE = -1;
//--------------------------Right-------------------------
//____________________�ٶȻ�__________________________
//����PID����
float Right_KP_SPEED = 2000;
float Right_KI_SPEED = 120;
float Right_KD_SPEED = 0;
//�������
float Right_DeadZone_SPEED = 0;
//���ַ��룬�����޷�
float Right_Separation_Threshold_SPEED = 10;
float Right_Integral_Limit_Max_SPEED = 8400;
float Right_Integral_Limit_Min_SPEED = -8400;
//����ܵ����
float Right_OutPut_Max_SPEED = 8400;
float Right_OutPut_Min_SPEED = -8400;
//____________________λ�û�__________________________
//����PID����
float Right_KP_DISTANCE = 6;
float Right_KI_DISTANCE = 0;
float Right_KD_DISTANCE = 5;
//�������
float Right_DeadZone_DISTANCE = 0.05;
//���ַ��룬�����޷�
float Right_Separation_Threshold_DISTANCE = 100;
float Right_Integral_Limit_Max_DISTANCE = 100000;
float Right_Integral_Limit_Min_DISTANCE = -100000;
//����ܵ����
float Right_OutPut_Max_DISTANCE = 1;
float Right_OutPut_Min_DISTANCE = -1;

//____________________�������һ�����__________________________
//����PID����
float ERROR_KP_DISTANCE = 4;
float ERROR_KI_DISTANCE = 0;
float ERROR_KD_DISTANCE = 0.2;
//�������
float ERROR_DeadZone_DISTANCE = 0;
//���ַ��룬�����޷�
float ERROR_Separation_Threshold_DISTANCE = 100;
float ERROR_Integral_Limit_Max_DISTANCE = 100000;
float ERROR_Integral_Limit_Min_DISTANCE = -100000;
//����ܵ����
float ERROR_OutPut_Max_DISTANCE = 1000;
float ERROR_OutPut_Min_DISTANCE = -10000;

//____________________�Ӿ�Ѳ�߻�__________________________
//����PID����
float FINDLINE_KP_DISTANCE = 0.004;//0.006
float FINDLINE_KI_DISTANCE = 0;
float FINDLINE_KD_DISTANCE = 0.08;
//�������
float FINDLINE_DeadZone_DISTANCE = 2;
//���ַ��룬�����޷�
float FINDLINE_Separation_Threshold_DISTANCE = 100;
float FINDLINE_Integral_Limit_Max_DISTANCE = 0.1;
float FINDLINE_Integral_Limit_Min_DISTANCE = -0.1;
//����ܵ����
float FINDLINE_OutPut_Max_DISTANCE = 1000;
float FINDLINE_OutPut_Min_DISTANCE = -10000;
//Ѳ�߻����ٶ��趨
float Base_Speed = 3.2;