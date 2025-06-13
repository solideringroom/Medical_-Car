#include "car_params_config.h"

//--------------------------Left-------------------------
//____________________速度环__________________________
//基础PID参数
float Left_KP_SPEED = 1800;//5000;
float Left_KI_SPEED = 120;//200
float Left_KD_SPEED = 0;//10000
//误差死区
float Left_DeadZone_SPEED = 0;
//积分分离，积分限幅
float Left_Separation_Threshold_SPEED = 10;
float Left_Integral_Limit_Max_SPEED = 8400;
float Left_Integral_Limit_Min_SPEED = -8400;
//最后总的输出
float Left_OutPut_Max_SPEED = 8400;
float Left_OutPut_Min_SPEED = -8400;
//____________________位置环__________________________
//基础PID参数
float Left_KP_DISTANCE = 3;
float Left_KI_DISTANCE = 0;
float Left_KD_DISTANCE = 0.01;
//误差死区
float Left_DeadZone_DISTANCE = 0.005;
//积分分离，积分限幅
float Left_Separation_Threshold_DISTANCE = 100;
float Left_Integral_Limit_Max_DISTANCE = 10000000;
float Left_Integral_Limit_Min_DISTANCE = -10000000;
//最后总的输出
float Left_OutPut_Max_DISTANCE = 1;
float Left_OutPut_Min_DISTANCE = -1;
//--------------------------Right-------------------------
//____________________速度环__________________________
//基础PID参数
float Right_KP_SPEED = 1800;
float Right_KI_SPEED = 120;
float Right_KD_SPEED = 0;
//误差死区
float Right_DeadZone_SPEED = 0;
//积分分离，积分限幅
float Right_Separation_Threshold_SPEED = 10;
float Right_Integral_Limit_Max_SPEED = 8400;
float Right_Integral_Limit_Min_SPEED = -8400;
//最后总的输出
float Right_OutPut_Max_SPEED = 8400;
float Right_OutPut_Min_SPEED = -8400;
//____________________位置环__________________________
//基础PID参数
float Right_KP_DISTANCE = 3;
float Right_KI_DISTANCE = 0;
float Right_KD_DISTANCE = 1;
//误差死区
float Right_DeadZone_DISTANCE = 0.005;
//积分分离，积分限幅
float Right_Separation_Threshold_DISTANCE = 100;
float Right_Integral_Limit_Max_DISTANCE = 100000;
float Right_Integral_Limit_Min_DISTANCE = -100000;
//最后总的输出
float Right_OutPut_Max_DISTANCE = 1;
float Right_OutPut_Min_DISTANCE = -1;

//____________________连接左右环的误差环__________________________
//基础PID参数
float ERROR_KP_DISTANCE = 4;
float ERROR_KI_DISTANCE = 0;
float ERROR_KD_DISTANCE = 0.2;
//误差死区
float ERROR_DeadZone_DISTANCE = 0;
//积分分离，积分限幅
float ERROR_Separation_Threshold_DISTANCE = 100;
float ERROR_Integral_Limit_Max_DISTANCE = 100000;
float ERROR_Integral_Limit_Min_DISTANCE = -100000;
//最后总的输出
float ERROR_OutPut_Max_DISTANCE = 1000;
float ERROR_OutPut_Min_DISTANCE = -10000;