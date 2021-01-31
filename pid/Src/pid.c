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
	  pid.SetSpeed = speed;     //�û��趨������ٶȴ���
	  
	  pid.err = pid.SetSpeed - pid.ActualSpeed;   //�������
	  
	  //���ַ���ͻ����޷�,ͬʱ������ʷƫ��
	  if(pid.ActualSpeed > pid.umax)
		{
		    if(abs(pid.err) > 200) //����һ��ֵ��ȡ����������
				{
				    index = 0;
				}
				else
				{
				    index = 1;
					  if(pid.err < 0) //��ǰֵ�����ֵ��ܶ࣬���뱥������ֻ�ۼӸ�ƫ��
						{
						    pid.Sk += pid.err;
						}
				}
				
		}
		else if(pid.ActualSpeed < pid.umin)
		{
		    if(abs(pid.err) > 200)  //����һ��ֵ��ȡ����������
				{
				    index = 0;
				}
				else
				{
				    index = 1;
					  if(pid.err > 0) //��ǰֵ����Сֵ��ܶ࣬���뱥������ֻ�ۼ���ƫ��
						{
						    pid.Sk += pid.err;
						}
				}
		}
		else
		{
		    index = 1;
			  if(abs(pid.err) > 200)  //����һ��ֵ��ȡ����������
				{
				    index = 0;
				}
				else
				{
				    index = 1;
					  pid.Sk += pid.err;
				}
		}
		
		//pid�������
		pid.out = pid.Kp * pid.err + index * pid.Ki * pid.Sk + pid.Kd * (pid.err - pid.err_last);
	  
		//�������
		pid.err_last = pid.err;
		
		//�õ���ǰ�ٶȣ�Ϊ�˲��Դ˴�ѡ��pid��������ֵ
		pid.ActualSpeed = pid.out * 1.0;
		
		
	  return pid.ActualSpeed;
}


