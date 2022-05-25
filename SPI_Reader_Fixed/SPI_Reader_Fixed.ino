// Quick and dirty Arduino code to read temps for the HVIs for FSAE LMS22

#include "Linduino.h"
#include "LT_SPI.h"
#include "LTC68042.h"
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <SPI.h>
#include <math.h>

// Pin definitions for Arduino
#define CS 10
#define MOSI 11
#define MISO 12
#define SCK 13

// Temperature structure
struct temperature {
	double max = 0.0;
	double avg = 0.0;
} raw, celsius;

// Just to make the code happy
const uint8_t TOTAL_IC = 1;

// Battery cell codes
uint16_t cell_codes[TOTAL_IC][12];
// GPIO codes
uint16_t aux_codes[TOTAL_IC][6];
// Configuration data to be written, stored in blocks of 6 bytes
uint8_t tx_cfg[TOTAL_IC][6];
// Configuration data read back, stored in blocks of 8 bytes
uint8_t rx_cfg[TOTAL_IC][8];

void run_command(uint16_t cmd) {
	int8_t error = 0;
	char input = 0;
	switch(cmd) {
		// Write configuration, used to turn on the reference and increase the speed of the ADC conversions
		case 1:
			wakeup_sleep();
			LTC6804_wrcfg(TOTAL_IC, tx_cfg);
			print_config();
			Serial.println();
			break;
		// Read configuration
		case 2:
			wakeup_sleep();
			error = LTC6804_rdcfg(TOTAL_IC, rx_cfg);
			if(error == -1) {
				Serial.println("A PEC error was detected in the received data, TmT");
			}
			print_rxconfig();
			Serial.println();
			break;
		// Start cell voltage conversion
		case 3:
			Serial.println("Starting cell conversion, UwU");
			wakeup_sleep();
			LTC6804_adcv();
			delay(3);
			Serial.println("Cell conversion completed, Ow<~");
			Serial.println();
			break;
		// Read cell voltages
		case 4:
			Serial.println("Reading cell voltages, :3");
			wakeup_sleep();
			error = LTC6804_rdcv(0, TOTAL_IC, cell_codes);
			if(error == -1) {
				Serial.println("A PEC error was detected in the received data, TmT");
			}
			print_cells();
			Serial.println();
			break;
		// Start auxiliary voltage conversion
		case 5:
			wakeup_sleep();
			LTC6804_adax();
			delay(3);
			Serial.println("Auxiliary conversion completed, Ow<~");
			Serial.println();
			break;
		// Read auxiliary voltages
		case 6:
			wakeup_sleep();
			error = LTC6804_rdaux(0, TOTAL_IC, aux_codes);
			if(error == -1) {
				Serial.println("A PEC error was detected in the received data, TmT");
			}
			print_aux();
			Serial.println();
			break;
		// Start cell voltage measurement loop
		case 7:
			Serial.println("Start measuring and printing cell voltages, o7");
			Serial.println("Press 'm' to quit, >wO");
			wakeup_sleep();
			LTC6804_wrcfg(TOTAL_IC, tx_cfg);
			while(input != 'm') {
				if(Serial.available() > 0) {
					input = Serial.read();
				}
				wakeup_idle();
				LTC6804_adcv();
				delay(10);
				wakeup_idle();
				error = LTC6804_rdcv(0, TOTAL_IC, cell_codes);
				if(error == -1) {
					Serial.println("A PEC error was detected in the received data, TmT");
				}
				print_cells();
				delay(500);
			}
			Serial.println();
			break;
		default:
			Serial.println("Please enter something between 1 and 7, UwU");
			print_menu();
			break;
	}
}

// Initialize the configuraiton array
void init_cfg() {
	tx_cfg[0][0] = 0xFE;
	tx_cfg[0][1] = 0x00;
	tx_cfg[0][2] = 0x00;
	tx_cfg[0][3] = 0x00;
	tx_cfg[0][4] = 0x00;
	tx_cfg[0][5] = 0x00;
}

// Print the menu options
void print_menu() {
	Serial.println("Please give me a command, Goshujin-Sama~");
	Serial.println("Write Configuration: 1");
	Serial.println("Read Configuration: 2");
	Serial.println("Start Cell Voltage Conversion: 3");
	Serial.println("Read Cell Voltages: 4");
	Serial.println("Start Aux Voltage Conversion: 5");
	Serial.println("Read Aux Voltages: 6");
	Serial.println("Loop cell voltages: 7");
	Serial.println("Command: ");
	Serial.println();
}

// Print cell voltage codes
void print_cells() {
	Serial.println("Begin printing cell codes~");
	for(int i = 0; i < 12; i++) {
	Serial.print(" C");
	Serial.print(i + 1, DEC);
	Serial.print(":");
	Serial.print(cell_codes[0][i] * 0.0001, 4);
	Serial.print(",");
	}
	Serial.println();
	Serial.println("Finished printing cell codes, UwU");
	Serial.println();
}

// Print GPIO voltage codes and Vref2 voltage code
void print_aux() {
	for(int i = 0; i < 5; i++) {
		Serial.print(" GPIO-");
		Serial.print(i + 1, DEC);
		Serial.print(":");
		Serial.print(aux_codes[0][i] * 0.0001, 4);
		Serial.print(",");
	}
	Serial.print(" Vref2");
	Serial.print(":");
	Serial.print(aux_codes[0][5] * 0.0001,4);
	Serial.println();
}

// Print the configuration data that is going to be written to the LTC6804 to the serial port
void print_config() {
	int cfg_pec;
	Serial.println("Written Configuration: ");
	Serial.print(": ");
	Serial.print("0x");
	serial_print_hex(tx_cfg[0][0]);
	Serial.print(", 0x");
	serial_print_hex(tx_cfg[0][1]);
	Serial.print(", 0x");
	serial_print_hex(tx_cfg[0][2]);
	Serial.print(", 0x");
	serial_print_hex(tx_cfg[0][3]);
	Serial.print(", 0x");
	serial_print_hex(tx_cfg[0][4]);
	Serial.print(", 0x");
	serial_print_hex(tx_cfg[0][5]);
	Serial.print(", Calculated PEC: 0x");
	cfg_pec = pec15_calc(6, &tx_cfg[0][0]);
	serial_print_hex((uint8_t) (cfg_pec >> 8));
	Serial.print(", 0x");
	serial_print_hex((uint8_t) (cfg_pec));
	Serial.println();
}

// Print the configuration data that was read back from the LTC6804 to the serial port
void print_rxconfig() {
	Serial.println("Received Configuration ");
	Serial.print(": 0x");
	serial_print_hex(rx_cfg[0][0]);
	Serial.print(", 0x");
	serial_print_hex(rx_cfg[0][1]);
	Serial.print(", 0x");
	serial_print_hex(rx_cfg[0][2]);
	Serial.print(", 0x");
	serial_print_hex(rx_cfg[0][3]);
	Serial.print(", 0x");
	serial_print_hex(rx_cfg[0][4]);
	Serial.print(", 0x");
	serial_print_hex(rx_cfg[0][5]);
	Serial.print(", Received PEC: 0x");
	serial_print_hex(rx_cfg[0][6]);
	Serial.print(", 0x");
	serial_print_hex(rx_cfg[0][7]);
	Serial.println();
}

// Print data in hex
void serial_print_hex(uint8_t data) {
	if(data < 16) {
		Serial.print("0");
		Serial.print((byte) data, HEX);
	} else {
		Serial.print((byte) data, HEX);
	}
}

// Setup routine
void setup() {
	Serial.begin(9600);
	LTC6804_initialize();
	init_cfg();
	print_menu();
}

// Main loop
void loop() {
	if(Serial.available()) {
		uint32_t user_command;
		user_command = Serial.read();
		user_command -= 48;
		Serial.println(user_command);
		run_command(user_command);
	}
}