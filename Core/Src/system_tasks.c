/*
 * system_tasks.c
 *
 *  Created on: Oct 27, 2025
 *      Author: najeh
 */


/* @brief This file contains the implementation for system tasks,
 */

#include "system_tasks.h"
#include "system_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For atof()
#include <math.h>   // For roundf()
#include "timers.h" // For software timers (xTimerCreate)
#include "queue.h"  // For FreeRTOS Queues
#include "jsmn.h"   // For JSON parsing
#include "main.h" // For access to HAL functions like HAL_GPIO_WritePin
#include "can_handle.h"
#include "jsmn.h"



// -----------------------------------------------------------
// FSM Types and Variables
// -----------------------------------------------------------
typedef enum {
    STATE_IDLE,
	STATE_CHECK,
    STATE_PREP_PATE,
    STATE_DILUTION,
    STATE_FINITION,
	STATE_EJECTION,
    STATE_RINCAGE,
} MachineState_t;

// FSM State Variable
static MachineState_t currentState = STATE_IDLE;

/**
 * @brief Helper function to compare a JSON token with a C-string.
 * @param json The full JSON string.
 * @param tok Pointer to the JSMN token.
 * @param s The C-string to compare against.
 * @return 0 if the token matches the string, non-zero otherwise.
 */
int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    return (tok->type == JSMN_STRING &&
            (int)strlen(s) == (tok->end - tok->start) &&
            strncmp(json + tok->start, s, tok->end - tok->start) == 0) ? 0 : -1;
}

// Helper function to safely change state
void setMachineState(MachineState_t newState) {
    if (currentState != newState) {
        currentState = newState;
        printf("FSM entering state: %d\n", currentState);
    }
}

// -----------------------------------------------------------
// 1. Task Handle Definitions (Allocate memory for the handles)
// These handles are declared 'extern' in system_tasks.h
// -----------------------------------------------------------
TaskHandle_t SensorHandle = NULL;
TaskHandle_t ControlHandle = NULL;
TaskHandle_t CommHandle = NULL;

// -----------------------------------------------------------
// 2. Queue Definition (The one-and-only definition)
// -----------------------------------------------------------
QueueHandle_t canParserQueue = NULL;

// -----------------------------------------------------------
// 3. Task Implementations
// -----------------------------------------------------------

/**
 * @brief FreeRTOS Task: High-priority data acquisition (Analog & Digital).
 * Runs periodically to keep g_SystemData fresh.
 */
void SensorAcquisitionTask(void *argument){
	while(1){
		// 1. Read and process Analog Sensors
		SYS_DATA_UpdateAnalog();

		// 2. Read Digital Sensors
		SYS_DATA_UpdateDigital();

		vTaskDelay(1000);
	}
}


/**
 * @brief FreeRTOS Task: The main Finite State Machine (FSM).
 * Runs periodically to check state and sensor data.
 */
void ControlLogicTask(void *argument){
	while(1){
		 // FSM logic
		switch (currentState) {
			case STATE_IDLE:
				 if (g_SystemData.potPresence_flag) {
				    setMachineState(STATE_PREP_PATE);
				 }
				break;

			case STATE_CHECK:
				// TODO: Add logic for CHECK state
				break;

			case STATE_PREP_PATE:
				// TODO: Add logic for PREP_PATE state
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

				break;

			case STATE_DILUTION:
				// TODO: Add logic for DILUTION state
				break;

			case STATE_FINITION:
				// TODO: Add logic for FINITION state
				break;

			case STATE_EJECTION:
				// TODO: Add logic for EJECTION state
				break;

			case STATE_RINCAGE:
				// TODO: Add logic for RINCAGE state
				break;

			default:
				// Unknown state, return to IDLE for safety
				setMachineState(STATE_IDLE);
				break;
		} // end switch

		vTaskDelay(500);
	}
}

/**
 * @brief FreeRTOS Task: CAN message parser.
 * Waits for messages on 'canParserQueue' and processes them.
 */
void CANParserTask(void *argument) {
    static char jsonMsg[1024];

	while (1) {
        // Wait forever for a message to arrive on the queue
		if (xQueueReceive(canParserQueue, &jsonMsg, portMAX_DELAY) == pdPASS) {
			printf("[CAN RX JSON] %s\n", jsonMsg);

			jsmn_parser parser;
			static jsmntok_t tokens[128];
			jsmn_init(&parser);
			int tokenCount = jsmn_parse(&parser, jsonMsg, strlen(jsonMsg), tokens, 128);

			if (tokenCount < 0) {
				printf("[JSON ERROR] %d\n", tokenCount);
				continue;
			}
			for (int i = 1; i < tokenCount; i++) {
				  if (jsoneq(jsonMsg, &tokens[i], "some_key") == 0) {
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

					  printf("msg received");
					  break;
				  }
				  }

		}
	}
}

/**
 * @brief FreeRTOS Task: Low-priority logging (The "Log Task").
 * Runs periodically to send data over CAN to the other nodes.
 */
void CommunicationTask(void *argument) {
	while(1) {
		char canRxBuf[30];
		snprintf(canRxBuf, sizeof(canRxBuf), "{\"some_key\":\"some_value\"}\n");
		if (xQueueSend(canParserQueue, canRxBuf, pdMS_TO_TICKS(10)) != pdPASS) {
				    printf("[CommTask] ERROR: Failed to send to queue\n");
				}
//		printf("water_temperature: %.5f\n",g_SystemData.pt100_water_temp);
		vTaskDelay(1000);
	}
}
