#ifndef __CAR_PARAMS_CONFIG_H
#define __CAR_PARAMS_CONFIG_H
//电机极性设置
#define LEFT_MOTOR_POLE -1
#define ROGHT_MOTOR_POLE 1

//--------------------------Left-------------------------
//____________________速度环__________________________
//基础PID参数
extern float Left_KP_SPEED;
extern float Left_KI_SPEED;
extern float Left_KD_SPEED;
//误差死区
extern float Left_DeadZone_SPEED;
//积分分离，积分限幅
extern float Left_Separation_Threshold_SPEED;
extern float Left_Integral_Limit_Max_SPEED;
extern float Left_Integral_Limit_Min_SPEED;
//最后总的输出
extern float Left_OutPut_Max_SPEED;
extern float Left_OutPut_Min_SPEED;
//____________________位置环__________________________
//基础PID参数
extern float Left_KP_DISTANCE;
extern float Left_KI_DISTANCE;
extern float Left_KD_DISTANCE;
//误差死区
extern float Left_DeadZone_DISTANCE;
//积分分离，积分限幅
extern float Left_Separation_Threshold_DISTANCE;
extern float Left_Integral_Limit_Max_DISTANCE;
extern float Left_Integral_Limit_Min_DISTANCE;
//最后总的输出
extern float Left_OutPut_Max_DISTANCE;
extern float Left_OutPut_Min_DISTANCE;
//--------------------------Right-------------------------
//____________________速度环__________________________
//基础PID参数
extern float Right_KP_SPEED;
extern float Right_KI_SPEED;
extern float Right_KD_SPEED;
//误差死区
extern float Right_DeadZone_SPEED;
//积分分离，积分限幅
extern float Right_Separation_Threshold_SPEED;
extern float Right_Integral_Limit_Max_SPEED;
extern float Right_Integral_Limit_Min_SPEED;
//最后总的输出
extern float Right_OutPut_Max_SPEED;
extern float Right_OutPut_Min_SPEED;
//____________________位置环__________________________
//基础PID参数
extern float Right_KP_DISTANCE;
extern float Right_KI_DISTANCE;
extern float Right_KD_DISTANCE;
//误差死区
extern float Right_DeadZone_DISTANCE;
//积分分离，积分限幅
extern float Right_Separation_Threshold_DISTANCE;
extern float Right_Integral_Limit_Max_DISTANCE;
extern float Right_Integral_Limit_Min_DISTANCE;
//最后总的输出
extern float Right_OutPut_Max_DISTANCE;
extern float Right_OutPut_Min_DISTANCE;

//____________________连接左右环的误差环__________________________
//基础PID参数
extern float ERROR_KP_DISTANCE ;
extern float ERROR_KI_DISTANCE ;
extern float ERROR_KD_DISTANCE ;
//误差死区
extern float ERROR_DeadZone_DISTANCE ;
//积分分离，积分限幅
extern float ERROR_Separation_Threshold_DISTANCE ;
extern float ERROR_Integral_Limit_Max_DISTANCE ;
extern float ERROR_Integral_Limit_Min_DISTANCE ;
//最后总的输出
extern float ERROR_OutPut_Max_DISTANCE ;
extern float ERROR_OutPut_Min_DISTANCE ;

//____________________视觉巡线环__________________________
//基础PID参数
extern float FINDLINE_KP_DISTANCE ;
extern float FINDLINE_KI_DISTANCE ;
extern float FINDLINE_KD_DISTANCE ;
//误差死区
extern float FINDLINE_DeadZone_DISTANCE ;
//积分分离，积分限幅
extern float FINDLINE_Separation_Threshold_DISTANCE ;
extern float FINDLINE_Integral_Limit_Max_DISTANCE ;
extern float FINDLINE_Integral_Limit_Min_DISTANCE ;
//最后总的输出
extern float FINDLINE_OutPut_Max_DISTANCE ;
extern float FINDLINE_OutPut_Min_DISTANCE ;
#endif