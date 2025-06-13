#include "car_control.h"
#include "soft_timer.h"
uint8_t RESET_DISTANCE_FLAG = 0;
extern float M2_SPEED;
float Left_Motor_Position_PID_Out,Left_Motor_Speed_PID_Out;
float Right_Motor_Position_PID_Out,Right_Motor_Speed_PID_Out;
float COMPO_ER_OUT;
Filter_Average ERROR_FILTER;//����˲���������˫��֮ǰ�����
positional_pid_ctrler ERROR_DISTANCE_PID;
#define ABS(x) ((x > 0) ? x : -x)

void ERROR_EARSER_INIT()
{
	positional_pid_controller_init(&ERROR_DISTANCE_PID,ERROR_KP_DISTANCE,ERROR_KI_DISTANCE,ERROR_KD_DISTANCE,ERROR_DeadZone_DISTANCE,ERROR_Separation_Threshold_DISTANCE,ERROR_Integral_Limit_Max_DISTANCE,ERROR_Integral_Limit_Min_DISTANCE,ERROR_OutPut_Max_DISTANCE,ERROR_OutPut_Min_DISTANCE);
	positional_pid_control(&ERROR_DISTANCE_PID,PID_ENABLE);
	
}
float EARSE_M1_M2_ERROR(uint8_t stright_or_turn)
{
	if(stright_or_turn == 0)
	{
		COMPO_ER_OUT = positional_pid_compute(&ERROR_DISTANCE_PID,Get_M1_Distance(),Get_M2_Distance(),&ERROR_FILTER);
	}
	else
	{
		COMPO_ER_OUT = positional_pid_compute(&ERROR_DISTANCE_PID,ABS(Get_M1_Distance()),ABS(Get_M2_Distance()),&ERROR_FILTER);
	}
	//COMPO_ER_OUT = 0.003*error_distance;
	return COMPO_ER_OUT;
}
//*********************������������֮�󷵻�0˵��û���꣬����1˵���ö�������������״̬�����ж��Ƿ��л���һ��״̬
//*********************����һ����������֮���Ƿ�Ҫ�ر�PID�����������ô�ٶȻ��Ļ��־�û�ˣ����ҿ�������������������Ļ��ز�����
//*********************ע�����涯������������ٶȺ�λ�Ƶ�λ����Ȧ�����Ǳ�����������
uint8_t Go_Forward(CarType* Car,float speed_rev,float distance)
{
	float speed = speed_rev;	
	//����ִ�������ǰ·�̣�ֱ�����ƽ���
	if(RESET_DISTANCE_FLAG == 0)
	{
		Reset_M1_Distance();
		Reset_M2_Distance();
		RESET_DISTANCE_FLAG = 1;
	}
	//�ٶȻ�PI��������λ�û�PD������
	//�ٶȵĿ��Ʒ�ʽ���޸�λ�û�������޷�����Ϊ��ʼ��ʱ��λ�û��������ͣ�����P����С��̬��D��ֹ�����
	//�޸����ұߵ��λ�û�������޷����ﵽ�������ٶȵ�Ŀ��
	Car->PID_Left_Motor_Distance.output_max = speed;
	Car->PID_Left_Motor_Distance.output_min = -speed;
	
	Car->PID_Right_Motor_Distance.output_max = speed;
	Car->PID_Right_Motor_Distance.output_min = -speed;
	//�������ҵ��λ�û�PID��������뵽�ٶȻ����������ٶȻ����
	Left_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Distance,distance,Get_M1_Distance(),&Car->PID_Left_Motor_Distance.FILTER_ERROR);
	Left_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Speed,Left_Motor_Position_PID_Out-EARSE_M1_M2_ERROR(0),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
	
	M2_SPEED = Get_M2_Speed();
	Right_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Distance,distance,Get_M2_Distance(),&Car->PID_Right_Motor_Distance.FILTER_ERROR);
	Right_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Speed,Right_Motor_Position_PID_Out+EARSE_M1_M2_ERROR(0),M2_SPEED,&Car->PID_Right_Motor_Speed.FILTER_ERROR);
	
	//�Ѽ��������ص�PWM���
	Load(Left_Motor_Speed_PID_Out,Right_Motor_Speed_PID_Out);
	
	//�����������λ��������������˵���ܽӽ�Ŀ���ˣ�ֱ��ֹͣ���������̱�־λ�ָ�,��PID�ر�
	if(Left_Motor_Position_PID_Out == 0 && Right_Motor_Position_PID_Out ==0)
	{
		Load(0,0);
		if(delay_timer(1,200))
		{
			RESET_DISTANCE_FLAG = 0;
			return 1;
		}
	}
	return 0;
}

uint8_t Go_Backward(CarType* Car,float speed_rev,float distance)
{
	//��ת�ٶȣ�·�̣���ΪҪ�����
	float speed = speed_rev;
	speed = -speed;
	distance = -distance;
	
	//����ִ�������ǰ·�̣�ֱ�����ƽ���
	if(RESET_DISTANCE_FLAG == 0)
	{
		Reset_M1_Distance();
		Reset_M2_Distance();
		RESET_DISTANCE_FLAG = 1;
	}
	//�ٶȻ�PI��������λ�û�PD������
	//�ٶȵĿ��Ʒ�ʽ���޸�λ�û�������޷�����Ϊ��ʼ��ʱ��λ�û��������ͣ�����P����С��̬��D��ֹ�����
	//�޸����ұߵ��λ�û�������޷����ﵽ�������ٶȵ�Ŀ��
	Car->PID_Left_Motor_Distance.output_max = -speed;
	Car->PID_Left_Motor_Distance.output_min = speed;
	
	Car->PID_Right_Motor_Distance.output_max = -speed;
	Car->PID_Right_Motor_Distance.output_min = speed;
	//�������ҵ��λ�û�PID��������뵽�ٶȻ����������ٶȻ����
	Left_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Distance,distance,Get_M1_Distance(),&Car->PID_Left_Motor_Distance.FILTER_ERROR);
	Left_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Speed,Left_Motor_Position_PID_Out-EARSE_M1_M2_ERROR(0),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
	
	Right_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Distance,distance,Get_M2_Distance(),&Car->PID_Right_Motor_Distance.FILTER_ERROR);
	Right_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Speed,Right_Motor_Position_PID_Out+EARSE_M1_M2_ERROR(0),Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
	
	//�Ѽ��������ص�PWM���
	Load(Left_Motor_Speed_PID_Out,Right_Motor_Speed_PID_Out);
	
	//�����������λ��������������˵���ܽӽ�Ŀ���ˣ�ֱ��ֹͣ���������̱�־λ�ָ�
	if(Left_Motor_Position_PID_Out == 0 && Right_Motor_Position_PID_Out ==0)
	{
		Load(0,0);
				if(delay_timer(2,200))
		{
			RESET_DISTANCE_FLAG = 0;
			return 1;
		}
	}
	return 0;
}

uint8_t Turn_Left(CarType* Car,float speed_rev,float distance)
{
	//��ת�ٶȣ�·�̣���ΪҪ�����
	float speed = speed_rev;
	float speed_left = -speed;
	float speed_right = speed;
	float distance_left = -distance;
	float distance_right = distance;
	
	//����ִ�������ǰ·�̣�ֱ�����ƽ���
	if(RESET_DISTANCE_FLAG == 0)
	{
		Reset_M1_Distance();
		Reset_M2_Distance();
		RESET_DISTANCE_FLAG = 1;
	}
	//�ٶȻ�PI��������λ�û�PD������
	//�ٶȵĿ��Ʒ�ʽ���޸�λ�û�������޷�����Ϊ��ʼ��ʱ��λ�û��������ͣ�����P����С��̬��D��ֹ�����
	//�޸����ұߵ��λ�û�������޷����ﵽ�������ٶȵ�Ŀ��
	Car->PID_Left_Motor_Distance.output_max = -speed_left;
	Car->PID_Left_Motor_Distance.output_min = speed_left;
	
	Car->PID_Right_Motor_Distance.output_max = speed_right;
	Car->PID_Right_Motor_Distance.output_min = -speed_right;
	//�������ҵ��λ�û�PID��������뵽�ٶȻ����������ٶȻ����
	Left_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Distance,distance_left,Get_M1_Distance(),&Car->PID_Left_Motor_Distance.FILTER_ERROR);
	Left_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Speed,Left_Motor_Position_PID_Out+EARSE_M1_M2_ERROR(1),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
	
	Right_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Distance,distance_right,Get_M2_Distance(),&Car->PID_Right_Motor_Distance.FILTER_ERROR);
	Right_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Speed,Right_Motor_Position_PID_Out+EARSE_M1_M2_ERROR(1),Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
	
	//�Ѽ��������ص�PWM���
	Load(Left_Motor_Speed_PID_Out,Right_Motor_Speed_PID_Out);
	
	//�����������λ��������������˵���ܽӽ�Ŀ���ˣ�ֱ��ֹͣ���������̱�־λ�ָ�
	if(Left_Motor_Position_PID_Out == 0 && Right_Motor_Position_PID_Out ==0)
	{
		Load(0,0);
		if(delay_timer(3,200))
		{
			RESET_DISTANCE_FLAG = 0;
			return 1;
		}
	}
	return 0;
}

uint8_t Turn_Right(CarType* Car,float speed_rev,float distance)
{
	//��ת�ٶȣ�·�̣���ΪҪ�����
	float speed = speed_rev;
	float speed_left = -speed;
	float speed_right = speed;
	float distance_left = distance;
	float distance_right = -distance;
	
	//����ִ�������ǰ·�̣�ֱ�����ƽ���
	if(RESET_DISTANCE_FLAG == 0)
	{
		Reset_M1_Distance();
		Reset_M2_Distance();
		RESET_DISTANCE_FLAG = 1;
	}
	//�ٶȻ�PI��������λ�û�PD������
	//�ٶȵĿ��Ʒ�ʽ���޸�λ�û�������޷�����Ϊ��ʼ��ʱ��λ�û��������ͣ�����P����С��̬��D��ֹ�����
	//�޸����ұߵ��λ�û�������޷����ﵽ�������ٶȵ�Ŀ��
	Car->PID_Left_Motor_Distance.output_max = -speed_left;
	Car->PID_Left_Motor_Distance.output_min = speed_left;
	
	Car->PID_Right_Motor_Distance.output_max = speed_right;
	Car->PID_Right_Motor_Distance.output_min = -speed_right;
	//�������ҵ��λ�û�PID��������뵽�ٶȻ����������ٶȻ����
	Left_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Distance,distance_left,Get_M1_Distance(),&Car->PID_Left_Motor_Distance.FILTER_ERROR);
	Left_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Speed,Left_Motor_Position_PID_Out-EARSE_M1_M2_ERROR(1),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
	
	Right_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Distance,distance_right,Get_M2_Distance(),&Car->PID_Right_Motor_Distance.FILTER_ERROR);
	Right_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Speed,Right_Motor_Position_PID_Out-EARSE_M1_M2_ERROR(1),Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
	
	//�Ѽ��������ص�PWM���
	Load(Left_Motor_Speed_PID_Out,Right_Motor_Speed_PID_Out);
	
	//�����������λ��������������˵���ܽӽ�Ŀ���ˣ�ֱ��ֹͣ���������̱�־λ�ָ�
	if(Left_Motor_Position_PID_Out == 0 && Right_Motor_Position_PID_Out ==0)
	{
		Load(0,0);
		if(delay_timer(4,200))
		{
			RESET_DISTANCE_FLAG = 0;
			return 1;
		}
	}
	return 0;
}


//�����PID������car_params_config.c��������
uint8_t Car_Init(CarType *Car)
{
	positional_pid_controller_init(&Car->PID_Left_Motor_Speed,Left_KP_SPEED,Left_KI_SPEED,Left_KD_SPEED,Left_DeadZone_SPEED,Left_Separation_Threshold_SPEED,Left_Integral_Limit_Max_SPEED,Left_Integral_Limit_Min_SPEED,Left_OutPut_Max_SPEED,Left_OutPut_Min_SPEED);
	positional_pid_controller_init(&Car->PID_Right_Motor_Speed,Right_KP_SPEED,Right_KI_SPEED,Right_KD_SPEED,Right_DeadZone_SPEED,Right_Separation_Threshold_SPEED,Right_Integral_Limit_Max_SPEED,Right_Integral_Limit_Min_SPEED,Right_OutPut_Max_SPEED,Right_OutPut_Min_SPEED);
	positional_pid_controller_init(&Car->PID_Left_Motor_Distance,Left_KP_DISTANCE,Left_KI_DISTANCE,Left_KD_DISTANCE,Left_DeadZone_DISTANCE,Left_Separation_Threshold_DISTANCE,Left_Integral_Limit_Max_DISTANCE,Left_Integral_Limit_Min_DISTANCE,Left_OutPut_Max_DISTANCE,Left_OutPut_Min_DISTANCE);
	positional_pid_controller_init(&Car->PID_Right_Motor_Distance,Right_KP_DISTANCE,Right_KI_DISTANCE,Right_KD_DISTANCE,Right_DeadZone_DISTANCE,Right_Separation_Threshold_DISTANCE,Right_Integral_Limit_Max_DISTANCE,Right_Integral_Limit_Min_DISTANCE,Right_OutPut_Max_DISTANCE,Right_OutPut_Min_DISTANCE);

	Encoder_Start();
	TB6612_Init();
	return 1;
}

void Car_Position_Mode_ON(CarType *Car)
{
	//�������������PID��˫����
	positional_pid_control(&Car->PID_Left_Motor_Speed,PID_ENABLE);
	positional_pid_control(&Car->PID_Left_Motor_Distance,PID_ENABLE);
	positional_pid_control(&Car->PID_Right_Motor_Speed,PID_ENABLE);
	positional_pid_control(&Car->PID_Right_Motor_Distance,PID_ENABLE);
}
void Car_Position_Mode_OFF(CarType *Car)
{
	//�ر����������PID��˫����
	positional_pid_control(&Car->PID_Left_Motor_Speed,PID_DISABLE);
	positional_pid_control(&Car->PID_Left_Motor_Distance,PID_DISABLE);
	positional_pid_control(&Car->PID_Right_Motor_Speed,PID_DISABLE);
	positional_pid_control(&Car->PID_Right_Motor_Distance,PID_DISABLE);
}	
