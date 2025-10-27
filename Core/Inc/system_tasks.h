/*
 * system_tasks.h
 *
 *  Created on: Oct 27, 2025
 *      Author: najeh
 */

#ifndef INC_SYSTEM_TASKS_H_
#define INC_SYSTEM_TASKS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// -----------------------------------------------------------
// External Task Handles (Defined in system_tasks.c)
// -----------------------------------------------------------
extern TaskHandle_t SensorHandle;
extern TaskHandle_t ControlHandle;
extern TaskHandle_t CommHandle;
// -----------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------
extern QueueHandle_t canParserQueue;


/**
 * @brief FreeRTOS Task: Data acquisition and sensor read (Analog & Digital).
 * @param argument Unused.
 */
void SensorAcquisitionTask(void *argument);

/**
 * @brief FreeRTOS Task: Execution of control algorithms and actuation.
 * @param argument Unused.
 */
void ControlLogicTask(void *argument);

/**
 * @brief FreeRTOS Task: CAN message parser.
 * Waits for messages on a queue and updates FSM/config.
 * @param argument Unused.
 */
void CANParserTask(void *argument);

/**
 * @brief FreeRTOS Task: Low-priority logging (The "Log Task").
 * @param argument Unused.
 */
void CommunicationTask(void *argument);

#endif /* INC_SYSTEM_TASKS_H_ */
