#ifndef PID_C_T
#define PID_C_T

#include <stdint.h>
#include <pid.h>
#include <stdio.h>
#include <bsp_can.h>

//限流
#define CURRENT_LIMIT 2000

extern int i;
extern int j;
extern int16_t IMU_Angle_Raw[3];   //接收到的陀螺仪欧拉角
extern float IMU_Angle[3];         //转化为浮点的陀螺仪欧拉角
extern PID_TypeDef abs_pid[2];           //绝对角度（自稳，陀螺仪）
extern PID_TypeDef motor_pid[4]; //速度环参数
extern PID_TypeDef angle_pid[2]; //位置环参数

void Abs_angle_control_loop();
void Rel_angle_control_loop();
void Abs_angle_PID_test_loop(int motor_ID);
void Rel_angle_PID_test_loop(int motor_ID);
void speed_loop_PID_tuning(int motor_ID);
void Rel_angleloop_PID_tuning(int motor_ID);
void Abs_anglelop_PID_tuning(int motor_ID);

#endif
