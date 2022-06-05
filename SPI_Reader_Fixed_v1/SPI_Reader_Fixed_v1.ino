// General Arduino code to read temps for the HVIs for FSAE LMS22
// Adapted from the Linduino demo programs

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LTC68042.h"
#include "Average.h"

#define TOTAL_IC 1

double maxTemperature = 0.0;
double totalVoltage = 0.0;
unsigned maxVoltage = 0;
long startTime = 0;

// Required arrays from demo code
uint16_t cell_codes[TOTAL_IC][12];
uint16_t aux_codes[TOTAL_IC][6];
uint8_t tx_cfg[TOTAL_IC][6];
uint8_t rx_cfg[TOTAL_IC][8];

void setup() {
	Serial.begin(9600);

	// Startup
	LTC6804_initialize();

	// Configuration bits
	init_cfg();
	delay(1000);

	startTime = millis();
}

void loop() {
	// Exit low-power mode
	wakeup_idle();

	// Start ADC
	LTC6804_adcv();

	// Wait for ADC to finish
	delay(10);
	wakeup_idle();

	// Read cell voltages
	if(LTC6804_rdcv(0, TOTAL_IC, cell_codes) == -1) {
		Serial.println("PEC Error");
	}

	// Convert voltage to Celsius
	getTemperature();
	printData();

	LTC6804_wrcfg(TOTAL_IC, tx_cfg);
	delay(1000);
}

// Functions for general integration
void LTCSetup() {
	// Startup
	LTC6804_initialize();
	// Configuration bits
	init_cfg();
	delay(1000);
}

void LTCLoop() {
	// Exit low-power mode
	wakeup_idle();
	// Start ADC
	LTC6804_adcv();
	// Wait for ADC to finish
	delay(10);
	wakeup_idle();
	// Read cell voltages
	LTC6804_rdcv(0, TOTAL_IC, cell_codes);
	// Convert voltage to Celsius
	getTemperature();
	LTC6804_wrcfg(TOTAL_IC, tx_cfg);
	delay(250);
}

// Function for setting configuration bits
void init_cfg() {
	for(int i = 0; i < TOTAL_IC; i++) {
		tx_cfg[i][0] = 0xFE;
		//tx_cfg[i][1] = 0x04;
		// 2.0 V
		tx_cfg[i][1] = 0x4E1;
		//tx_cfg[i][2] = 0xE1;
		// 3.6 V
		tx_cfg[i][2] = 0x8CA;
		tx_cfg[i][3] = 0x00;
		tx_cfg[i][4] = 0x00;
		// tx_cfg[i][5] = 0x00;
		// sets the software timer to 1 minute
		tx_cfg[i][5] = 0x20;
	}
}

// Function for printing all relevant data
void printData() {
	long elapsedTime = millis() - startTime;
	maxVoltage = cell_codes[0][0];

	// Time elapsed
	Serial.print("Time Elapsed (ms): ");
	Serial.print(elapsedTime);
	Serial.print("\t");

	// Cell voltages
	Serial.print("Cell Voltages 1 - 9 (V): ");
	for(int i = 0 ; i < TOTAL_IC; i++) {
		totalVoltage = 0.0;
		for(int j = 0; j < 9; j++) {
			totalVoltage += cell_codes[i][j] * 0.0001;
			if(cell_codes[i][j] < maxVoltage) {
				maxVoltage = cell_codes[i][j];
			}
			Serial.print(cell_codes[i][j] * 0.0001, 4);
			Serial.print(", ");
		}
	}
	Serial.print("\t");

	// Highest voltage
	Serial.print("Max Voltage (V): ");
	Serial.print(maxVoltage * 0.0001, 4);
	Serial.print("\t");

	// Total voltage
	Serial.print("Total Voltage (V): ");
	Serial.print(totalVoltage, 4);
	Serial.print("\t");

	// Temperature
	Serial.print("Temperature (C): ");
	Serial.print(maxTemperature);
	Serial.print("\t");

	Serial.print("\n");
}

// Function for converting voltage to Celsius
void getTemperature() {
	// y = mx + b
	// m = -7140.054127
	// b = 23468.21191
	// From linear regression
	maxTemperature = ((-7140) * maxVoltage) + 23468;
}