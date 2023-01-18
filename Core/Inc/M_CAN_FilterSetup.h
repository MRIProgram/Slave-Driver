/*
 * M_CAN_FilterSetup.h
 *
 *  Created on: Dec 4, 2022
 *      Author: ismarintan
 */

#ifndef INC_M_CAN_FILTERSETUP_H_
#define INC_M_CAN_FILTERSETUP_H_

#include "main.h"
#include "can.h"


typedef struct
{

	CAN_FilterTypeDef FilterConf;


} M_CAN_Filter_Typedef;

void M_CAN_Filter_Init(M_CAN_Filter_Typedef *pFilt_Cfg, CAN_HandleTypeDef *hCAN);


#endif /* INC_M_CAN_FILTERSETUP_H_ */
