/*
 * obd2.c
 *
 *  Created on: Nov 1, 2023
 *      Author: legob
 */

#include "obd2.h"

static uint32_t last_rx;
static float current_speed;

uint8_t can_data[8];
uint8_t can_len;

HAL_StatusTypeDef obd2_init(FDCAN_HandleTypeDef *hcan)
{
	HAL_StatusTypeDef status;
	FDCAN_FilterTypeDef filter;

	last_rx = HAL_GetTick();
	current_speed = 0;

	filter.IdType = FDCAN_STANDARD_ID;
	filter.FilterIndex = 0;
	filter.FilterType = FDCAN_FILTER_DUAL;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = OBD2_RESPONSE_ID;
	filter.FilterID2 = OBD2_REQUEST_ID;

	status = HAL_FDCAN_ConfigFilter(hcan, &filter);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_FDCAN_Start(hcan);

	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	if (status != HAL_OK)
	{
		return status;
	}

	status = obd2_request_speed(hcan);

	return status;
}

void obd2_handle_rx(FDCAN_HandleTypeDef *hcan)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    OBD2_MSG *obd2_msg = (OBD2_MSG *) data;

    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK)
    {
        return;
    }

    can_len = rx_header.DataLength >> 16;
    can_data[0] = data[0];
    can_data[1] = data[1];
    can_data[2] = data[2];
    can_data[3] = data[3];

    if (rx_header.Identifier != OBD2_RESPONSE_ID)
    {
        return;
    }

    if (rx_header.DataLength != OBD2_RESPONSE_LENGTH << 16)
    {
    	return;
    }

    if (obd2_msg->dlen != OBD2_RESPONSE_DLEN)
    {
    	return;
    }

    if (obd2_msg->mode != OBD2_RESPONSE_MODE)
    {
    	return;
    }

    if (obd2_msg->pid != OBD2_RESPONSE_PID)
    {
    	return;
    }

    current_speed = 0.6213712 * obd2_msg->a;

    last_rx = HAL_GetTick();
}

HAL_StatusTypeDef obd2_request_speed(FDCAN_HandleTypeDef *hcan)
{
    uint32_t start = HAL_GetTick();

    FDCAN_TxHeaderTypeDef header;

    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.Identifier = OBD2_REQUEST_ID;
    header.DataLength = OBD2_REQUEST_LENGTH << 16;

    uint8_t data[OBD2_REQUEST_LENGTH];
    OBD2_MSG *obd2_msg = (OBD2_MSG *) data;

    obd2_msg->dlen = OBD2_REQUEST_DLEN;
    obd2_msg->mode = OBD2_REQUEST_MODE;
    obd2_msg->pid = OBD2_REQUEST_PID;

    while (HAL_FDCAN_GetTxFifoFreeLevel(hcan) == 0)
    {
        if (HAL_GetTick() - start > OBD2_REQUEST_TIMEOUT_MS)
        {
            break;
        }
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data);
}

float obd2_get_speed(void)
{
	return current_speed;
}

HAL_StatusTypeDef obd2_get_status(void)
{
	if (HAL_GetTick() - last_rx > OBD2_RESPONSE_TIMEOUT_MS)
	{
		return HAL_TIMEOUT;
	}
	else
	{
		return HAL_OK;
	}
}
