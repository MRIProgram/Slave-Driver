/*
 * M_PID.c
 *
 *  Created on: Dec 3, 2022
 *      Author: ismarintan
 */


#include "M_PID.h"

void M_PID_Calculate(M_PID_Typedef *pPID, float SetPoint, float Refference,
		float *PID_Out)
{
	pPID->Error = SetPoint - Refference;

	pPID->SumError += pPID->Error * pPID->TimeSampling;

	if (pPID->ModePID == M_PID_MAX_I_WINDUP)
	{
		if (pPID->SumError > pPID->Max_Output)
			pPID->SumError = pPID->Max_Output;
		else if (pPID->SumError < -pPID->Max_Output)
			pPID->SumError = -pPID->Max_Output;
	}

	pPID->Proportional = pPID->Kp * pPID->Error;
	pPID->Integral = pPID->Ki * pPID->SumError;
	pPID->Derivative = pPID->Kd * (pPID->Error - pPID->PrevError)
			/ pPID->TimeSampling;

	pPID->PrevError = pPID->Error;

	float Output = pPID->Proportional + pPID->Integral + pPID->Derivative;

	if (Output > pPID->Max_Output)
		Output = pPID->Max_Output;
	else if (Output < -pPID->Max_Output)
		Output = -pPID->Max_Output;

	*PID_Out = Output;

}
