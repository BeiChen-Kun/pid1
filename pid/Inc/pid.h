#ifndef _PID_H_
#define _PID_H_
#include "main.h"


typedef struct
{
	float SetSpeed;		  	    //�û��趨�ٶ�
	float ActualSpeed;  		  //��ǰ�ٶ�
	
	float err;                //��ǰ���
	float err_last;           //�ϴ����
	
	float Kp;   		        	//����ϵ��
	float Ki;                 //����ʱ�䳣��
	float Kd;                 //΢��ʱ�䳣��
	
	float Sk;                 //��ʷƫ��֮��
	
	float out;                //PID�������
	//float Iout;               //����ֵ
	
	float umax;               //����ֵ���ֵ
	float umin;               //����ֵ��Сֵ
	
	//float limit;              //�������

}PID;

extern PID pid;

void PID_Init(void);      // PID��ʼ��
float PID_Calculate(float speed); // pid����


#endif
