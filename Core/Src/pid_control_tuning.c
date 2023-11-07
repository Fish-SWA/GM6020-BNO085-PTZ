#include "pid_control_tuning.h"

PID_TypeDef motor_pid[4]; //速度环参数
PID_TypeDef angle_pid[2]; //位置环参数


PID_TypeDef abs_pid[2];           //绝对角度（自稳，陀螺仪）
float ABS_Gimbal_angle[2] = {0};  //云台绝对角度(自稳) {yaw, pitch}, 0-8192 3400-6000
int16_t ABS_IMU_angle[2] = {0};   //归一化之后的IMU角度，用于自稳 {yaw, pitch}, 0-8192
int16_t IMU_Angle_Raw[3] = {0};   //接收到的陀螺仪欧拉角
float IMU_Angle[3] = {0};         //转化为浮点的陀螺仪欧拉角

int16_t Gimbal_angle[2] = {7500, 4800};   //云台角度设定 {yaw, pitch}，编码器


int i = 0;
int j = 0;
int current = 0;
int speed = 0;
int time = 0;
int speed_set = 0;

//陀螺仪位置控制
void Abs_angle_control_loop()
{
  /*自稳控制*/
  //IMU归一化
  ABS_IMU_angle[0] = (int)((IMU_Angle[2]*8192)/360); //yaw
  ABS_IMU_angle[1] = (int)(((IMU_Angle[0]+180.0)*8192)/360); //pitch
  //YAW计算
  int motor_ID = 0;
  abs_pid[motor_ID].target = ABS_Gimbal_angle[0];
  abs_pid[motor_ID].f_cal_pid(&abs_pid[motor_ID], ABS_IMU_angle[motor_ID], 8192);
  motor_pid[motor_ID].target = abs_pid[motor_ID].output; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

  //PITCH计算
  motor_ID = 1;
  abs_pid[motor_ID].target = ABS_Gimbal_angle[1];
  abs_pid[motor_ID].f_cal_pid(&abs_pid[motor_ID], ABS_IMU_angle[motor_ID], 8192);
  motor_pid[motor_ID].target = abs_pid[motor_ID].output; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);


  set_moto_current(&hcan1, motor_pid[0].output, motor_pid[1].output, 0, 0);
  printf("%d, %d\n", moto_chassis[motor_ID].angle, ABS_IMU_angle[motor_ID]);
  HAL_Delay(1);
}

//编码器位置控制
void Rel_angle_control_loop()
{
    
    i++;
  /*自稳计算*/
  //IMU归一化
  ABS_IMU_angle[0] = (int)(IMU_Angle[2]*(8192/360)); //yaw
  ABS_IMU_angle[1] = (int)((IMU_Angle[0]+180.0)*(8192/360)); //pitch

  if(Gimbal_angle[0] > 8191) Gimbal_angle[0] = 0;
  else if(Gimbal_angle[0] < 0) Gimbal_angle[0] = 8191;
    //用PID计算电流
    //这里用for会有奇怪的问题，暂时展开
    int motor_ID = 0;
    angle_pid[motor_ID].target = Gimbal_angle[0];
    angle_pid[motor_ID].f_cal_pid(&angle_pid[motor_ID], moto_chassis[motor_ID].angle, 8192);
    motor_pid[motor_ID].target = angle_pid[motor_ID].output;
    motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

    motor_ID = 1;
    angle_pid[motor_ID].target = Gimbal_angle[1];
    angle_pid[motor_ID].f_cal_pid(&angle_pid[motor_ID], moto_chassis[motor_ID].angle, 8192);
    motor_pid[motor_ID].target = angle_pid[motor_ID].output; 																							
    motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

    set_moto_current(&hcan1, motor_pid[0].output, motor_pid[1].output, 0, 0);
    printf("%d, %d\n", moto_chassis[0].angle, moto_chassis[1].angle);
    printf("%f\n", IMU_Angle[0]);
    HAL_Delay(1); //1000hz
}

//自稳PID效果测试
void Abs_angle_PID_test_loop(int motor_ID)
{
  //自稳效果测试
  //IMU归一化
  ABS_IMU_angle[0] = (int)((IMU_Angle[2]*8192)/360); //yaw
  ABS_IMU_angle[1] = (int)(((IMU_Angle[0]+180.0)*8192)/360); //pitch
  abs_pid[motor_ID].target = 0;
  abs_pid[motor_ID].f_cal_pid(&abs_pid[motor_ID], ABS_IMU_angle[motor_ID], 8192);

  motor_pid[motor_ID].target = abs_pid[motor_ID].output; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

  if(motor_ID == 0) set_moto_current(&hcan1, motor_pid[motor_ID].output, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, motor_pid[motor_ID].output, 0, 0);
  //set_moto_current(&hcan1, 0, 0, 0, 0);
  //printf("%d\n", moto_chassis[0].speed_rpm);
  //printf("%d\n", moto_chassis[0].total_angle);
  //printf("%d, %f, %f\n", ABS_IMU_angle[motor_ID], abs_pid[motor_ID].output, motor_pid[motor_ID].output);
  printf("%d, %d\n", moto_chassis[motor_ID].angle, ABS_IMU_angle[motor_ID]);
  HAL_Delay(1);
}

//位置环PID效果测试
void Rel_angle_PID_test_loop(int motor_ID)
{
  angle_pid[motor_ID].target = 7000;
  angle_pid[motor_ID].f_cal_pid(&angle_pid[motor_ID], moto_chassis[motor_ID].angle, 8192);
  //printf("%d\n", (int)angle_pid.output);

  motor_pid[motor_ID].target = angle_pid[motor_ID].output; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

  if(motor_ID == 0) set_moto_current(&hcan1, motor_pid[motor_ID].output, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, motor_pid[motor_ID].output, 0, 0);
  //set_moto_current(&hcan1, 0, 0, 0, 0);
  //printf("%d\n", moto_chassis[0].speed_rpm);
  //printf("%d\n", moto_chassis[0].total_angle);
  printf("%f, ", (float)j);
  HAL_Delay(1);
  printf("%d, %d, %d, %d, %f, %f, %d\n", 
                    speed_set, moto_chassis[motor_ID].angle, 
                    (int)motor_pid[motor_ID].output, 
                    (int)motor_pid[motor_ID].iout, 
                    angle_pid[motor_ID].output, 
                    IMU_Angle[motor_ID]);
}

//速度环科学调参（编码器角速度）
void speed_loop_PID_tuning(int motor_ID)
{
  speed = moto_chassis[motor_ID].speed_rpm;
  time++;
  if(i <= 400) current = 0;
  else current = 1500;

  if(i >= 800) i = 0;

  printf("%f, ", (float)j);
  printf("%d, %d\n", current, speed);
  if(motor_ID == 0) set_moto_current(&hcan1, current, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, current, 0, 0);
}

//编码器位置环科学调参
void Rel_angleloop_PID_tuning(int motor_ID)
{
  if(i <= 100) speed_set = 0;
  else if(i >100 && i<=1100) speed_set = -15;
  else if(i >1100 && i<=2100) speed_set = 15;
  if(i > 2100) i = 0;
  //speed_set = 25;
  
  motor_pid[motor_ID].target = speed_set; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID], moto_chassis[motor_ID].speed_rpm, 0);
  if(motor_ID == 0) set_moto_current(&hcan1, motor_pid[motor_ID].output, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, motor_pid[motor_ID].output, 0, 0);
  //set_moto_current(&hcan1, 0, 0, 0, 0);
  //printf("%d\n", moto_chassis[0].speed_rpm);
  //printf("%d\n", moto_chassis[0].total_angle);
  printf("%f, ", (float)j);
  printf("%d, %d\n", speed_set, moto_chassis[motor_ID].total_angle);
}

//陀螺仪位置环科学调参
void Abs_anglelop_PID_tuning(int motor_ID)
{
  //IMU归一化
  ABS_IMU_angle[0] = (int)((IMU_Angle[2]*8192)/360); //yaw
  ABS_IMU_angle[1] = (int)(((IMU_Angle[0]+180.0)*8192)/360); //pitch

  if(i <= 100) speed_set = 0;
  else if(i >100 && i<=1100) speed_set = -15;
  else if(i >1100 && i<=2100) speed_set = 15;
  if(i > 2100) i = 0;
  //speed_set = 25;
  
  motor_pid[motor_ID].target = speed_set; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID], moto_chassis[motor_ID].speed_rpm, 0);
  if(motor_ID == 0) set_moto_current(&hcan1, motor_pid[motor_ID].output, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, motor_pid[motor_ID].output, 0, 0);
  //set_moto_current(&hcan1, 0, 0, 0, 0);
  //printf("%d\n", moto_chassis[0].speed_rpm);
  //printf("%d\n", moto_chassis[0].total_angle);
  printf("%f, ", (float)j);
  printf("%d, %d\n", speed_set, ABS_IMU_angle[motor_ID]);
}

