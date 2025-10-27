#ifndef INC_CAN_HANDLE_H_
#define INC_CAN_HANDLE_H_

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// Constants
#define CAN_RX_MAX_LEN 1024

// Extern Queue used for CAN message parser
// This is DECLARING that the queue exists somewhere else (defined in system_tasks.c)
extern QueueHandle_t canParserQueue;

// -----------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------

/**
 * @brief Sends a complete message string over CAN with StdId 0x122.
 * The function will chunk the message into 8-byte frames
 * and append a newline character.
 * @param msg The null-terminated string to send.
 */
void CAN_SendJsonMessage(const char *msg);

/**
 * @brief Sends a complete message string over CAN with StdId 0x446.
 * (Used for other CAN communication)
 * @param msg The null-terminated string to send.
 */
void CAN_Send(const char *msg);

/**
 * @brief HAL CAN Receive FIFO 0 Message Pending Callback.
 * This is the ISR that runs when a CAN frame is received.
 * @param hcan Pointer to the CAN handle.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif /* INC_CAN_HANDLE_H_ */
