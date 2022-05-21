// Quick and dirty Arduino code to read temps for the HVIs for FSAE LMS22

#include <SPI.h>
#include <math.h>

#define CSB 10
#define MOSI 11
#define MISO 12
#define SCK 13

// Read and write commands for SPI
const byte READ = 0b11111100;
const byte WRITE = 0b00000010;

// Temperature variables
double average_temp;
int max_temp;
int16_t cells[9];
int16_t PECs[9];

// Frequency of the LTC
const long int freq = 100000;

// Voltage to temperature look-up table
int volt_to_temp[255];

// Loop-up table stuff
int16_t pec15Table[256];
int16_t CRC15_POLY = 0x4599;

void init_PEC15_Table() {
	int16_t remainder;
	for(int i = 0; i < 256; i++) {
		remainder = i << 7;
		for(int bit = 8; bit > 0; --bit) {
			if(remainder & 0x4000) {
				remainder = remainder << 1;
				remainder = pow(remainder, CRC15_POLY);
			}
			else {
				remainder = ((remainder << 1));
			}
		}
		pec15Table[i] = remainder & 0xFFFF;
	}
}

uint16_t lookup_pec15(uint16_t data) {
	int16_t remainder, address;
	// PEC seed
	remainder = 16;
	for(int i = 0; i < 16; i++) {
		// Calculate PEC table address
		address = pow((remainder >> 7), ((data >> i) & 1));
    address = address & 0xFF;
		remainder = pow((remainder << 8 ), pec15Table[address]);
	}
	// The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
	return (remainder * 2);
}

// This function will send the command to each LTC to poll every cell and read cells 1 through 9
int pollAndRead() {
	uint16_t data = uint16_t(0x0306); //Casting the hexadecimal value to a short int so the lookup works smoothly
	SPI.transfer16(0x0306); //Poll all cells, without discharge, and on the 7kHz mode.
	SPI.transfer16(lookup_pec15(data)); //Sent PEC

	/*
	* The miso line will be held low while the ADC is underway, and will be set to high
	* when polling is finished. This will take about 2-3 ms, and the serial line may not 
	* be used for other things in the meantime.
	*/
	// (MISO used to be 10)

	SPI.transfer16(0x0004); //Read group A cell voltages
	SPI.transfer16(lookup_pec15(uint16_t(0x0004))); //PEC

	//After sending the read command, the data will be sent back to the Arduino
	//as V[7],...,V[0],V[15],...,V[8]
	
	cells[0] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	cells[1] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	cells[2] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	PECs[0] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	PECs[1] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	PECs[2] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;

  /*
  Serial.print("The PEC 0 lookup is ");
  Serial.print(lookup_pec15(uint16_t(cells[3])));
  Serial.println();
  Serial.print("The PEC 0 is ");
  Serial.print(PECs[3]);
  Serial.println();

	if(PECs[0] != lookup_pec15(uint16_t(cells[0])))  return 1;
	if(PECs[1] != lookup_pec15(uint16_t(cells[1])))  return 1;
	if(PECs[2] != lookup_pec15(uint16_t(cells[2])))  return 1;

	//If the parity error codes do not match, return a 1, indicating an error
	*/
	SPI.transfer16(0x0006); //Read group B cell voltages
	SPI.transfer16(lookup_pec15(uint16_t(0x0006))); //PEC

	//After sending the read command, the data will be sent back to the Arduino
	//as V[7],...,V[0],V[15],...,V[8]
	
	cells[3] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	cells[4] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	cells[5] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	PECs[3] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	PECs[4] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	PECs[5] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
  /*
	if(PECs[3] != lookup_pec15(uint16_t(cells[3])))  return 1;
	if(PECs[4] != lookup_pec15(uint16_t(cells[4])))  return 1;
	if(PECs[5] != lookup_pec15(uint16_t(cells[5])))  return 1;

	//If the parity error codes do not match, return a 1, indicating an error
  */
	SPI.transfer16(0x0008); //Read group C cell voltages
	SPI.transfer16(lookup_pec15(uint16_t(0x0008))); //PECs

	//After sending the read command, the data will be sent back to the Arduino
	//as V[7],...,V[0],V[15],...,V[8]
	
	cells[6] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	cells[7] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	cells[8] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	PECs[6] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	PECs[7] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
	PECs[8] = SPI.transfer(0x0) + SPI.transfer(0x0) << 8;
  /*
	if(PECs[6] != lookup_pec15(uint16_t(cells[6])))  return 1;
	if(PECs[7] != lookup_pec15(uint16_t(cells[7])))  return 1;
	if(PECs[8] != lookup_pec15(uint16_t(cells[8])))  return 1;
  */
	//If the parity error codes do not match, return a 1, indicating an error

	//If no errors are detected, return 0.
	return 0;
}

// More temp functions
void updateTemps() {
	int temporal_temp;
	for(int i = 0; i < 9; i++) {
		if(cells[i] > 255) {
			temporal_temp = 1000;
		} else {
			temporal_temp = int(cells[i] / 100);
			temporal_temp = volt_to_temp[temporal_temp];
		}
		if(temporal_temp > max_temp) {
			max_temp = temporal_temp;
		}
		average_temp = (average_temp + temporal_temp) / 90;
	}
	//Finally, reset the arrays holding the values for cell voltages and PECs
	for(int i = 0; i < 9; i++) {
		cells[i] = 0;
		PECs[i] = 0;
	}
}

void setup() {
	//First of all, the temperature array is filled following a trendline approximation to the data on the datasheet
	for(int i = 0; i < 256; i++) {
		if(i < 137) {
			volt_to_temp[i] = 200;
		} else if(i > 235) { 
			volt_to_temp[i] = -200;
		} else {
			volt_to_temp[i] = int(0.004359 * i * i - 2.6123 * i + 358.79);
		}
	}
	// SPI pins
	pinMode(CSB, OUTPUT);
	// Initialize all selectors to make their devices ignore their MOSIs 
	digitalWrite(CSB, HIGH);
  delay(100);
	Serial.begin(9600);
	while(!Serial);
	Serial.println("Serial On!");
  digitalWrite(CSB, LOW);
	SPI.beginTransaction(SPISettings(freq, MSBFIRST, SPI_MODE3));
	// Select one HVI board at a time
	// We use the configuration instruction defined previously
  SPI.transfer(0x00);
  SPI.transfer(0x01);
  SPI.transfer(0x3D);
  SPI.transfer(0x6E);
	SPI.endTransaction();
	Serial.println("Boop, SPI config done!");
	delay(100);
}

void loop() {
  digitalWrite(CSB, LOW);
	SPI.beginTransaction(SPISettings(freq, MSBFIRST, SPI_MODE3));
	// The program scans the LTC's to measure each cell's temperature
	// Select one HVI board at a time
	// If there is an error measuring any cell in a board, the count will be adjusted and the board will be 
	// checked again
  SPI.transfer(0x00);
  SPI.transfer(0x04);
  SPI.transfer(0x07);
  SPI.transfer(0xC2);
  delay(100);
    
  pollAndRead();
  delay(100);
	updateTemps();
  delay(100);
  SPI.transfer(0x07);
  SPI.transfer(0x11);
  SPI.transfer(0xC9);
  SPI.transfer(0xC0);
  delay(100);
  SPI.transfer(0x9F);
  SPI.transfer(0x14);
  SPI.transfer(0x1C);
  SPI.transfer(0x48);
  delay(100);
	SPI.endTransaction();
	//Deselect the HVI board and reset the U/L select
	digitalWrite(CSB, HIGH);
  delay(100);
	//Print stuff to terminal
	Serial.print("The max temperature is ");
  Serial.print(max_temp);
  Serial.println();
	Serial.print("The average temperature is ");
  Serial.print(average_temp, 5);
  Serial.println();
}
