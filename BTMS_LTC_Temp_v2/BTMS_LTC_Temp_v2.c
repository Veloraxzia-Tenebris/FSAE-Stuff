// General Arduino code to read temps for the HVIs for FSAE LMS22
// Adapted from the Linduino demo programs
// Remade to fit all of LMS22's weird quirks

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

#include "BTMS_LTC_Temp_v2.h"

// Liduino.h
//! @param pin pin to be driven LOW
#define output_low(pin)   digitalWrite(pin, LOW)
//! Set "pin" high
//! @param pin pin to be driven HIGH
#define output_high(pin)  digitalWrite(pin, HIGH)

// LTC68042.h
#define MD_NORMAL 2
#define DCP_DISABLED 0
#define CELL_CH_ALL 0
#define AUX_CH_ALL 0

// LTC68042.cpp
/*!
  6804 conversion command variables.
*/
uint8_t ADCV[2]; //!< Cell Voltage conversion command.
uint8_t ADAX[2]; //!< GPIO conversion command.

#define TOTAL_IC 1

#define SCK 9
#define MOSI 8

double maxTemperature = 0.0;
double totalVoltage = 0.0;
unsigned maxVoltage = 0;
long startTime = 0;

// Required arrays from demo code
uint16_t cell_codes[TOTAL_IC][12];
uint16_t aux_codes[TOTAL_IC][6];
uint8_t tx_cfg[TOTAL_IC][6];
uint8_t rx_cfg[TOTAL_IC][8];

// Functions for general integration
void LTCSetup(uint8_t LTCPin) {
	// Startup
	pinMode(SCK, OUTPUT);             //! 1) Setup SCK as output
	pinMode(MOSI, OUTPUT);            //! 2) Setup MOSI as output
	pinMode(LTCPin, OUTPUT);     //! 3) Setup CS as output
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV32);
	uint8_t md_bits;
	uint8_t MD = MD_NORMAL;
	uint8_t DCP = DCP_DISABLED;
	uint8_t CH = CELL_CH_ALL;
	uint8_t CHG = AUX_CH_ALL;
	md_bits = (MD & 0x02) >> 1;
	ADCV[0] = md_bits + 0x02;
	md_bits = (MD & 0x01) << 7;
	ADCV[1] =  md_bits + 0x60 + (DCP<<4) + CH;
	md_bits = (MD & 0x02) >> 1;
	ADAX[0] = md_bits + 0x04;
	md_bits = (MD & 0x01) << 7;
	ADAX[1] = md_bits + 0x60 + CHG;

	// Configuration bits
	init_cfg();
	delay(1000);
}

void LTCLoop(uint8_t LTCPin) {
	// Exit low-power mode
	output_high(LTCPin);
	delayMicroseconds(10); //Guarantees the isoSPI will be in ready mode
	output_low(LTCPin);

	// Start ADC
	uint8_t cmd[4];
	uint16_t temp_pec;
	//1
	cmd[0] = ADCV[0];
	cmd[1] = ADCV[1];
	//2
	temp_pec = pec15_calc(2, ADCV);
	cmd[2] = (uint8_t)(temp_pec >> 8);
	cmd[3] = (uint8_t)(temp_pec);
	//3
	output_high(LTCPin);
	delayMicroseconds(10); //Guarantees the isoSPI will be in ready mode
	output_low(LTCPin); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
	//4
	output_low(LTCPin);
	spi_write_array(4,cmd);
	output_high(LTCPin);

	// Wait for ADC to finish
	delay(10);
	output_high(LTCPin);
	delayMicroseconds(10); //Guarantees the isoSPI will be in ready mode
	output_low(LTCPin);

	// Read cell voltages
	uint8_t reg = 0;
	uint8_t total_ic = TOTAL_IC;
	const uint8_t NUM_RX_BYT = 8;
	const uint8_t BYT_IN_REG = 6;
	const uint8_t CELL_IN_REG = 3;
	uint8_t *cell_data;
	int8_t pec_error = 0;
	uint16_t parsed_cell;
	uint16_t received_pec;
	uint16_t data_pec;
	uint8_t data_counter=0; //data counter
	cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
	//1.a
	if (reg == 0)
	{
	//a.i
	for (uint8_t cell_reg = 1; cell_reg<5; cell_reg++)               //executes once for each of the LTC6804 cell voltage registers
	{
		data_counter = 0;
		LTC6804_rdcv_reg(cell_reg, total_ic,cell_data, LTCPin);
		for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++) // executes for every LTC6804 in the stack
		{
		// current_ic is used as an IC counter
		//a.ii
		for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)                  // This loop parses the read back data. Loops
		{
			// once for each cell voltages in the register
			parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);
			cell_codes[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
			data_counter = data_counter + 2;
		}
		//a.iii
		received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1];
		data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT ]);
		if (received_pec != data_pec)
		{
			pec_error--;//pec_error = -1;
		}
		data_counter=data_counter+2;
		}
	}
	}
	//1.b
	else
	{
	//b.i
	LTC6804_rdcv_reg(reg, total_ic,cell_data, LTCPin);
	for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++) // executes for every LTC6804 in the stack
	{
		// current_ic is used as an IC counter
		//b.ii
		for (uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)                    // This loop parses the read back data. Loops
		{
		// once for each cell voltage in the register
		parsed_cell = cell_data[data_counter] + (cell_data[data_counter+1]<<8);
		cell_codes[current_ic][current_cell + ((reg - 1) * CELL_IN_REG)] = 0x0000FFFF & parsed_cell;
		data_counter= data_counter + 2;
		}
		//b.iii
		received_pec = (cell_data[data_counter] << 8 )+ cell_data[data_counter + 1];
		data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT * (reg-1)]);
		if (received_pec != data_pec)
		{
		pec_error--;//pec_error = -1;
		}
	}
	}
	free(cell_data);
	//2
	return(pec_error);

	// Convert voltage to Celsius
	getTemperature();
	uint8_t total_ic = TOTAL_IC;
	uint8_t config[][6] = tx_cfg;
	const uint8_t BYTES_IN_REG = 6;
	const uint8_t CMD_LEN = 4+(8*total_ic);
	uint8_t *cmd;
	uint16_t temp_pec;
	uint8_t cmd_index; //command counter
	cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));
	//1
	cmd[0] = 0x00;
	cmd[1] = 0x01;
	cmd[2] = 0x3d;
	cmd[3] = 0x6e;
	//2
	cmd_index = 4;
	for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each LTC6804 in stack,
	{
	for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each byte in the CFGR register
	{
		// i is the byte counter

		cmd[cmd_index] = config[current_ic-1][current_byte];    //adding the config data to the array to be sent
		cmd_index = cmd_index + 1;
	}
	//3
	temp_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]);// calculating the PEC for each board
	cmd[cmd_index] = (uint8_t)(temp_pec >> 8);
	cmd[cmd_index + 1] = (uint8_t)temp_pec;
	cmd_index = cmd_index + 2;
	}
	//4
	output_high(LTCPin);
	delayMicroseconds(10); //Guarantees the isoSPI will be in ready mode
	output_low(LTCPin);                                //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
	//5
	for (int current_ic = 0; current_ic<total_ic; current_ic++)
	{
	cmd[0] = 0x80 + (current_ic<<3); //Setting address
	temp_pec = pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(temp_pec >> 8);
	cmd[3] = (uint8_t)(temp_pec);
	output_high(LTCPin);
	spi_write_array(4,cmd);
	spi_write_array(8,&cmd[4+(8*current_ic)]);
	output_low(LTCPin);
	}
	free(cmd);
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

uint16_t pec15_calc(uint8_t len, uint8_t *data) {
  uint16_t remainder,addr;
  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
    remainder = (remainder<<8)^crc15Table[addr];
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
  for (uint8_t i = 0; i < len; i++)
  {
    spi_write((char)data[i]);
  }
}

void spi_write(int8_t  data)  // Byte to be written to SPI port
{
  SPDR = data;                  //! 1) Start the SPI transfer
  while (!(SPSR & _BV(SPIF)));  //! 2) Wait until transfer complete
}

void LTC6804_rdcv_reg(uint8_t reg,
                      uint8_t total_ic,
                      uint8_t *data,
				  uint8_t LTCPin
                     )
{
  uint8_t cmd[4];
  uint16_t temp_pec;

  //1
  if (reg == 1)
  {
    cmd[1] = 0x04;
    cmd[0] = 0x00;
  }
  else if (reg == 2)
  {
    cmd[1] = 0x06;
    cmd[0] = 0x00;
  }
  else if (reg == 3)
  {
    cmd[1] = 0x08;
    cmd[0] = 0x00;
  }
  else if (reg == 4)
  {
    cmd[1] = 0x0A;
    cmd[0] = 0x00;
  }

  //2


  //3
	output_high(LTCPin);
	delayMicroseconds(10); //Guarantees the isoSPI will be in ready mode
	output_low(LTCPin); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4
  for (int current_ic = 0; current_ic<total_ic; current_ic++)
  {
    cmd[0] = 0x80 + (current_ic<<3); //Setting address
    temp_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(temp_pec >> 8);
    cmd[3] = (uint8_t)(temp_pec);
    output_high(LTCPin);
    spi_write_read(cmd,4,&data[current_ic*8],8);
    output_low(LTCPin);
  }
}