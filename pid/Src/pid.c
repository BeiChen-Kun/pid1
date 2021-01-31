#include "pid.h"
//#include <iostream>
#include <stdlib.h>
PID pid;

void PID_Init(void)
{
    pid.SetSpeed = 0.0;
	  pid.ActualSpeed = 0.0;
	  pid.err = 0.0;
	  pid.err_last = 0.0;
	  pid.out = 0.0;
	  pid.Sk = 0.0;
	  pid.Kp = 0.2;
	  pid.Ki = 0.1;
	  pid.Kd = 0.2;
	  pid.umax = 400;
	  pid.umin = -200;
	  //pid.limit = 200.0;

}

float PID_Calculate(float speed)
{
	  int index;
	  pid.SetSpeed = speed;     //用户设定输入的速度传入
	  
	  pid.err = pid.SetSpeed - pid.ActualSpeed;   //计算误差
	  
	  //积分分离和积分限幅,同时计算历史偏差
	  if(pid.ActualSpeed > pid.umax)
		{
		    if(abs(pid.err) > 200) //误差超过一定值，取消积分作用
				{
				    index = 0;
				}
				else
				{
				    index = 1;
					  if(pid.err < 0) //当前值比最大值大很多，进入饱和区，只累加负偏差
						{
						    pid.Sk += pid.err;
						}
				}
				
		}
		else if(pid.ActualSpeed < pid.umin)
		{
		    if(abs(pid.err) > 200)  //误差超过一定值，取消积分作用
				{
				    index = 0;
				}
				else
				{
				    index = 1;
					  if(pid.err > 0) //当前值比最小值大很多，进入饱和区，只累加正偏差
						{
						    pid.Sk += pid.err;
						}
				}
		}
		else
		{
		    index = 1;
			  if(abs(pid.err) > 200)  //误差超过一定值，取消积分作用
				{
				    index = 0;
				}
				else
				{
				    index = 1;
					  pid.Sk += pid.err;
				}
		}
		
		//pid计算输出
		pid.out = pid.Kp * pid.err + index * pid.Ki * pid.Sk + pid.Kd * (pid.err - pid.err_last);
	  
		//更新误差
		pid.err_last = pid.err;
		
		//得到当前速度，为了测试此处选择pid计算的输出值
		pid.ActualSpeed = pid.out * 1.0;
		
		
	  return pid.ActualSpeed;
}


