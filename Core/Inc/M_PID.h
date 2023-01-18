/*
 * M_PID.h
 *
 *  Created on: Dec 3, 2022
 *      Author: ismarintan
 *
 *
 *  Version 1.0
 *
 */





#ifndef INC_M_PID_H_
#define INC_M_PID_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

typedef enum
{
	M_PID_MODE_NORMAL = 0, M_PID_MAX_I_WINDUP = 1

} M_PID_Mode;

typedef struct
{

	float Proportional, Derivative, Integral;
	float Error, SumError, PrevError;

	float Kp, Ki, Kd;
	float Max_Output;

	M_PID_Mode ModePID;

	float TimeSampling; // Dalam Detik

} M_PID_Typedef;

void M_PID_Calculate(M_PID_Typedef *pPID, float SetPoint, float Refference,
		float *PID_Out);



#ifdef __cplusplus
}
#endif
#endif /* INC_M_PID_H_ */
