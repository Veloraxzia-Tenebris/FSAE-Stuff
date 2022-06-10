// General Arduino code to read temps for the HVIs for FSAE LMS22
// Adapted from the Linduino demo programs
// Remade to fit all of LMS22's weird quirks

// DON'T OVERRIDE THESE GLOBALS
/*
	#define TOTAL_IC 1
	double maxTemperature = 0.0;
	double totalVoltage = 0.0;
	unsigned maxVoltage = 0;
	long startTime = 0;
	uint16_t cell_codes[TOTAL_IC][12];
	uint16_t aux_codes[TOTAL_IC][6];
	uint8_t tx_cfg[TOTAL_IC][6];
	uint8_t rx_cfg[TOTAL_IC][8];
*/

#include "Linduino.h"
#include "LT_SPI.h"
#include "SERCOM.h"

// Put this in setup
// Function for configuring and setting up the LTC
void LTCSetup(uint8_t);

// Put this in loop
// Function to read voltages and convert temperatures
// maxTemperature is updated per call of this function with the hottest temperature
double LTCLoop(uint8_t);

// Function to set up initialization configuration for the LTC
void init_cfg();

// Function to convert the temperature using global variables
void getTemperature();

// Function
uint16_t pec15_calc(uint8_t, uint8_t*);

void LTC6804_rdcv_reg(uint8_t, uint8_t, uint8_t, uint8_t);

void wakeup_idle(uint8_t);

void LTC6804_adcv(uint8_t);

uint8_t LTC6804_rdcv(uint8_t reg, uint8_t total_ic, uint16_t cell_codes[][12], uint8_t);

void LTC6804_wrcfg(uint8_t nIC,uint8_t config[][6], uint8_t);

void spi_write_read(uint8_t *TxData, uint8_t TXlen, uint8_t *rx_data, uint8_t RXlen);

void spi_write_array( uint8_t length, uint8_t *data);