/*
 * system_config.h
 *
 *  Created on: Oct 24, 2025
 *      Author: najeh
 */

#ifndef INC_SYSTEM_CONFIG_H_
#define INC_SYSTEM_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

// Static varibales declaration
#define NUM_ADC_CHANNELS 4


// Structure for all system data
typedef struct {
    // Analog Sensors
    float pt100_tank_temp;
    float pt100_water_temp;
    float ph_value;
    float hpt_value;

    // Digital Sensors
    bool processWaterMin_flag;
    bool processWaterMax_flag;
    bool waterMin_flag;
    bool waterMax_flag;
    bool potPresence_flag;
    bool potMin_flag;
} SystemData_t;

// -----------------------------------------------------------
// 1. External Declaration of the Global Data Structure
extern SystemData_t g_SystemData;
extern uint16_t adc_buffer[NUM_ADC_CHANNELS];

// -----------------------------------------------------------
// 2. Function Prototypes for Data Access/Update

// Initializes the structure with safe default values
void SYS_DATA_Init(void);

// Functions to read sensors values (called by the analog update function)
float READ_pH_SENSOR(void);
float READ_WATER_TEMP(void);
float READ_TANK_TEMP(void);
float READ_OIL_LEVEL(void);
// Functions to update sensor readings (called by the sensor task)
void SYS_DATA_UpdateAnalog(void);
void SYS_DATA_UpdateDigital(void);

#endif /* INC_SYSTEM_CONFIG_H_ */
