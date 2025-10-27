/*
 * system_config.c
 *
 * Created on: Oct 24, 2025
 * Author: najeh
 *
 * @brief This file implements the functions for initializing and updating
 * the central SystemData_t structure, handling sensor conversions
 * from raw ADC values to physical units, and reading digital inputs.
 */


#include "system_config.h"
#include "stm32f4xx_hal.h"
#include "main.h" // Includes peripheral handles and GPIO defines (e.g., waterMin_Pin)

// -----------------------------------------------------------
// 1. Definition of the Global Data Structure
// -----------------------------------------------------------
/**
 * @brief Global structure instance holding the current state of all sensors and actuators.
 * Declared as 'extern' in system_config.h and defined here.
 */
SystemData_t g_SystemData;

/**
 * @brief Buffer to store the raw 12-bit ADC values read via DMA.
 * The order corresponds to the ADC channel configuration sequence:
 * [0] -> PT100_Tank_Temp
 * [1] -> PT100_Water_Temp
 * [2] -> pH_Sensor
 * [3] -> HPT (Oil Level)
 */
uint16_t adc_buffer[NUM_ADC_CHANNELS];

// -----------------------------------------------------------
// 2. Function Implementations
// -----------------------------------------------------------

/**
 * @brief Initializes the global system data structure to safe, known default values.
 * Should be called once during system startup (after HAL_Init).
 */
void SYS_DATA_Init(void) {
    // Analog Sensors Initialization
    g_SystemData.pt100_tank_temp = 0.0f;
    g_SystemData.pt100_water_temp = 0.0f;
    g_SystemData.ph_value = 7.0f; // Default to neutral pH
    g_SystemData.hpt_value = 0.0f; // Default to zero level/pressure

    // Digital Sensors Initialization
    g_SystemData.processWaterMin_flag = false;
    g_SystemData.processWaterMax_flag = false;
    g_SystemData.waterMin_flag = false;
    g_SystemData.waterMax_flag = false;
    g_SystemData.potPresence_flag = false;
    g_SystemData.potMin_flag = false;
}

/**
 * @brief Converts raw ADC value for the pH Sensor to pH units.
 * @note  Calibration formula: pH = 7.0 * V_measured.
 * Assumes VDD_A = 3.3V and a 12-bit ADC (4095 max value).
 * The measurement range is capped at 2.0V to prevent over-scaling.
 * @return The calculated pH value (float).
 */
float READ_pH_SENSOR(void){
	uint16_t adc_value = adc_buffer[2];
	float voltage = (adc_value / 4095.0f) * 3.3f;

	// Apply a cap on the voltage to prevent erroneous readings from scaling
	if (voltage > 2.0f) {
        voltage = 2.0f;
    }

	// Linearly scale voltage to pH (pH = 7 * Voltage)
	float pH = 7.0f * voltage;

	return pH;
}

/**
 * @brief Converts raw ADC value for the Water Temperature Sensor (PT100) to degrees Celsius.
 * @note  Assumes a 4-20mA sensor with a 150 Ohm shunt resistor (V = I * R = 20mA * 150 Ohm = 3.0V max).
 * The temperature range is assumed to be -50°C to 200°C (range 250°C).
 * Scaling: Temp = (((I_mA - 4.0) / 16.0) * 250.0) - 50.0
 * @return The calculated temperature in °C (float).
 */
float READ_WATER_TEMP(void){
	uint16_t adc_value = adc_buffer[1];
	float voltage = (adc_value / 4095.0f) * 3.3f;

    // Convert Voltage (V_shunt) to Current (mA)
	float current_mA = (voltage / 150.0f) * 1000.0f; // V / 150 Ohm * 1000 = mA

    // Convert 4-20mA to Temperature range (-50C to 200C)
	float temperature = ((current_mA - 4.0f) / 16.0f) * 250.0f - 50.0f;

	return temperature;
}

/**
 * @brief Converts raw ADC value for the Tank Temperature Sensor (PT100) to degrees Celsius.
 * @note  Same scaling/assumptions as the Water Temperature Sensor.
 * @return The calculated temperature in °C (float).
 */
float READ_TANK_TEMP(void){
	uint16_t adc_value = adc_buffer[0];
	float voltage = (adc_value / 4095.0f) * 3.3f;

    // Convert Voltage (V_shunt) to Current (mA)
	float current_mA = (voltage / 150.0f) * 1000.0f; // V / 150 Ohm * 1000 = mA

    // Convert 4-20mA to Temperature range (-50C to 200C)
	float temperature = ((current_mA - 4.0f) / 16.0f) * 250.0f - 50.0f;

	return temperature;
}

/**
 * @brief Converts raw ADC value for the HPT (High-Pressure/Level Transmitter) to Oil Level.
 * @note  Assumes a 4-20mA sensor.
 * The level range is assumed to be 0 to 13 units (e.g., cm or inches).
 * Scaling: Level = ((I_mA - 4.0) / 16.0) * 13.0
 * @return The calculated oil level (float).
 */
float READ_OIL_LEVEL(void){
	uint16_t adc_value = adc_buffer[3];
	float voltage = (adc_value / 4095.0f) * 3.3f;

    // Convert Voltage (V_shunt) to Current (mA)
	float current_mA = (voltage / 150.0f) * 1000.0f; // V / 150 Ohm * 1000 = mA

    // Convert 4-20mA to Oil Level range (0 to 13 units)
	float oil_level = ((current_mA - 4.0f) / 16.0f) * 13.0f;

	return oil_level;
}

/**
 * @brief Reads the raw ADC data from the DMA buffer and converts it into physical units,
 * storing the results in the global SystemData_t structure.
 * @note  This function relies on the ADC and DMA being running in the background
 * to ensure the 'adc_buffer' contains fresh data.
 */
void SYS_DATA_UpdateAnalog(void) {
	// Note: The index corresponds to the ADC channel Rank/Sequence configured in MX_ADC1_Init
	g_SystemData.pt100_tank_temp = READ_TANK_TEMP();    // Rank 1, Buffer Index 0
    g_SystemData.pt100_water_temp = READ_WATER_TEMP();  // Rank 2, Buffer Index 1
    g_SystemData.ph_value = READ_pH_SENSOR();          // Rank 3, Buffer Index 2
    g_SystemData.hpt_value = READ_OIL_LEVEL();         // Rank 4, Buffer Index 3
}

/**
 * @brief Reads all digital sensor states (limit switches, presence sensors)
 * directly from the configured GPIO pins and stores them in the structure.
 * @note  Assumes that all GPIOs are configured as inputs.
 * Active HIGH (GPIO_PIN_SET) indicates the condition is met, unless specified.
 */
void SYS_DATA_UpdateDigital(void) {

    // Water level sensors
    g_SystemData.waterMin_flag = (HAL_GPIO_ReadPin(waterMin_GPIO_Port, waterMin_Pin) == GPIO_PIN_SET);
    g_SystemData.waterMax_flag = (HAL_GPIO_ReadPin(waterMax_GPIO_Port, waterMax_Pin) == GPIO_PIN_SET);

    // Pot presence/minimum level sensors
    // Note: potPresence is configured as active LOW (RESET)
    g_SystemData.potPresence_flag = (HAL_GPIO_ReadPin(potPresence_GPIO_Port, potPresence_Pin) == GPIO_PIN_RESET);
    g_SystemData.potMin_flag = (HAL_GPIO_ReadPin(potMin_GPIO_Port, potMin_Pin)== GPIO_PIN_SET);

    // Process water min/max sensors
    g_SystemData.processWaterMax_flag = (HAL_GPIO_ReadPin(processWaterMax_GPIO_Port, processWaterMax_Pin) == GPIO_PIN_SET);
    g_SystemData.processWaterMin_flag = (HAL_GPIO_ReadPin(processWaterMin_GPIO_Port, processWaterMin_Pin) == GPIO_PIN_SET);
}
