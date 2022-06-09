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

// Put this in setup
// Function for configuring and setting up the LTC
void LTCSetup();

// Put this in loop
// Function to read voltages and convert temperatures
// maxTemperature is updated per call of this function with the hottest temperature
void LTCLoop();