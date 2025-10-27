#include "main.h" // For HAL and hcan1
#include <string.h>
#include <stdio.h> // For printf (debugging)
#include "can_handle.h"

// -----------------------------------------------------------
// External Handles (Defined elsewhere)
// -----------------------------------------------------------

// CAN handle defined in main.c
extern CAN_HandleTypeDef hcan1;

// -----------------------------------------------------------
// File-Static Variables
// -----------------------------------------------------------

// Static buffer to reassemble incoming CAN frames into a string
// Its size is defined in can_handle.h
static char canRxBuf[CAN_RX_MAX_LEN];
// Current index in the reassembly buffer
static uint32_t canRxIdx = 0;

/**
 * @brief HAL CAN Receive FIFO 0 Message Pending Callback (ISR).
 * @note  This function executes in interrupt context.
 * It reassembles 8-byte CAN frames into a full string.
 * When it finds a newline ('\n'), it sends the complete
 * string to the 'canParserQueue'.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data);

    for (int i = 0; i < rxHeader.DLC; i++) {
        char c = data[i];

        if (canRxIdx < CAN_RX_MAX_LEN - 1) {
            canRxBuf[canRxIdx++] = c;

            if (c == '\n') {
                canRxBuf[canRxIdx] = '\0';
                if (xQueueSendFromISR(canParserQueue, canRxBuf, &xHigherPriorityTaskWoken) != pdPASS) {
                    ITM_SendChar('E'); // queue error
                }
                canRxIdx = 0;
                memset(canRxBuf, 0, sizeof(canRxBuf));
            }
        } else {
            canRxIdx = 0;
            memset(canRxBuf, 0, sizeof(canRxBuf));
            ITM_SendChar('O'); // overflow
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Transmits a long string over CAN by chunking it into 8-byte frames.
 * Appends a '\n' to mark the end of the message.
 * Uses CAN ID 0x122.
 * @param msg The null-terminated JSON string to send.
 */
void CAN_SendJsonMessage(const char *msg)
{
	 CAN_TxHeaderTypeDef txHeader;
	    uint32_t txMailbox;
	    HAL_StatusTypeDef status;

	    txHeader.StdId = 0x122;
	    txHeader.IDE = CAN_ID_STD;
	    txHeader.RTR = CAN_RTR_DATA;

	    int len = strlen(msg);
	    int totalLen = len + 1;  // +1 for '\n'
	    const char *ptr = msg;

	    for (int i = 0; i < totalLen; i += 8) {
	        uint8_t chunk[8] = {0};

	        for (int j = 0; j < 8 && (i + j) < totalLen; j++) {
	            if ((i + j) < len)
	                chunk[j] = ptr[i + j];
	            else
	                chunk[j] = '\n';  // Append newline at the end
	        }

	        txHeader.DLC = ((totalLen - i) >= 8) ? 8 : (totalLen - i);

	        // Retry with timeout
	        uint32_t timeout = 1000;
	        do {
	            status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, chunk, &txMailbox);
	            vTaskDelay(10);
	        } while (status == HAL_BUSY && --timeout > 0);

	        if (status != HAL_OK) {
	            // Print to ITM/SWV console
	            printf("[CAN ERROR] Failed to send CAN frame. Status: %d, Timeout left: %lu\n", status, timeout);
	        }
	    }
}

/**
 * @brief Transmits a long string over CAN with StdId 0x446.
 * (Used for other CAN communication)
 * @param msg The null-terminated string to send.
 */
void CAN_Send(const char *msg)
{
	 CAN_TxHeaderTypeDef txHeader;
	    uint32_t txMailbox;
	    HAL_StatusTypeDef status;

	    txHeader.StdId = 0x446;
	    txHeader.IDE = CAN_ID_STD;
	    txHeader.RTR = CAN_RTR_DATA;

	    int len = strlen(msg);
	    int totalLen = len + 1;  // +1 for '\n'
	    const char *ptr = msg;

	    for (int i = 0; i < totalLen; i += 8) {
	        uint8_t chunk[8] = {0};

	        for (int j = 0; j < 8 && (i + j) < totalLen; j++) {
	            if ((i + j) < len)
	                chunk[j] = ptr[i + j];
	            else
	                chunk[j] = '\n';  // Append newline at the end
	        }

	        txHeader.DLC = ((totalLen - i) >= 8) ? 8 : (totalLen - i);

	        // Retry with timeout
	        uint32_t timeout = 1000;
	        do {
	            status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, chunk, &txMailbox);
	            vTaskDelay(10);
	        } while (status == HAL_BUSY && --timeout > 0);

	        if (status != HAL_OK) {
	            // Print to ITM/SWV console
	            printf("[CAN ERROR] Failed to send CAN frame. Status: %d, Timeout left: %lu\n", status, timeout);
	        }
	    }
}

