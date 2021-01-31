#ifndef _PID_H_
#define _PID_H_
#include "main.h"


typedef struct
{
	float SetSpeed;		  	    //用户设定速度
	float ActualSpeed;  		  //当前速度
	
	float err;                //当前误差
	float err_last;           //上次误差
	
	float Kp;   		        	//比例系数
	float Ki;                 //积分时间常数
	float Kd;                 //微分时间常数
	
	float Sk;                 //历史偏差之和
	
	float out;                //PID的总输出
	//float Iout;               //积分值
	
	float umax;               //误差极限值最大值
	float umin;               //误差极限值最小值
	
	//float limit;              //误差限制

}PID;

extern PID pid;

void PID_Init(void);      // PID初始化
float PID_Calculate(float speed); // pid计算


#endif
