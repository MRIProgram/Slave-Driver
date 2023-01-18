/*
 * M_CAN_FilterSetup.c
 *
 *  Created on: Dec 4, 2022
 *      Author: ismarintan
 */


#include "M_CAN_FilterSetup.h"


void M_CAN_Filter_Init(M_CAN_Filter_Typedef *pFilt_Cfg, CAN_HandleTypeDef *hCAN)
{

	pFilt_Cfg->FilterConf.FilterBank = 0;
	pFilt_Cfg->FilterConf.FilterMode = CAN_FILTERMODE_IDMASK;
	pFilt_Cfg->FilterConf.FilterScale = CAN_FILTERSCALE_32BIT;

	pFilt_Cfg->FilterConf.FilterIdHigh = 0x0000;
	pFilt_Cfg->FilterConf.FilterIdLow = 0x0000;

	pFilt_Cfg->FilterConf.FilterMaskIdHigh = 0x0000;
	pFilt_Cfg->FilterConf.FilterMaskIdLow = 0x0000;
	pFilt_Cfg->FilterConf.FilterFIFOAssignment = CAN_RX_FIFO0;

	pFilt_Cfg->FilterConf.FilterActivation = ENABLE;
	pFilt_Cfg->FilterConf.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(hCAN, &pFilt_Cfg->FilterConf);
	HAL_CAN_ActivateNotification(hCAN, CAN_IT_RX_FIFO0_MSG_PENDING);


}
