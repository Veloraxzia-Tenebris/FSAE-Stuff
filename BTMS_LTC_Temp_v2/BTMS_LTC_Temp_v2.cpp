// General Arduino code to read temps for the HVIs for FSAE LMS22
// Adapted from the Linduino demo programs
// Remade to fit all of LMS22's weird quirks

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
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
static const unsigned int crc15Table[256] = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
    0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
    0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
    0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
    0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
    0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
    0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
    0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
    0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
    0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
    0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
    0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
    0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
    0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
    0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
    0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
    0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
    0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
    0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
    0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
    0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
    0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
    0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
                                            };

// LTC68042.cpp
/*!
  6804 conversion command variables.
*/
uint8_t ADCV[2]; //!< Cell Voltage conversion command.
uint8_t ADAX[2]; //!< GPIO conversion command.

#define TOTAL_IC 1

//#define SCK 9
//#define MOSI 8

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
	spi_enable(SPI_CLOCK_DIV32);
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
	delay(10);
}

double LTCLoop(uint8_t LTCPin) {
	double temp = 0.0;
	// General setup
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
	delay(10);

	// Exit low-power mode
	wakeup_idle(LTCPin);

	// Start ADC
	LTC6804_adcv(LTCPin);

	// Wait for ADC to finish
	delay(10);
	wakeup_idle(LTCPin);

	// Read cell voltages
	LTC6804_rdcv(0, TOTAL_IC, cell_codes, LTCPin);
	// Convert voltage to Celsius
	temp = getTemperature();
	LTC6804_wrcfg(TOTAL_IC, tx_cfg, LTCPin);
	delay(10);

	return temp;
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
double getTemperature() {
	// y = mx + b
	// m = -7140.054127
	// b = 23468.21191
	// From linear regression
	maxTemperature = ((-7140) * maxVoltage) + 23468;
	return maxTemperature;
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

void wakeup_idle(uint8_t LTCPin)
{
  output_high(LTCPin);
  delayMicroseconds(10); //Guarantees the isoSPI will be in ready mode
  output_low(LTCPin);
}

void LTC6804_adcv(uint8_t LTCPin)
{

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
  wakeup_idle (LTCPin); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  //4
  output_high(LTCPin);
  spi_write_array(4,cmd);
  output_low(LTCPin);

}

uint8_t LTC6804_rdcv(uint8_t reg,
                     uint8_t total_ic,
                     uint16_t cell_codes[][12],
				 uint8_t LTCPin
                    )
{

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
}

void LTC6804_wrcfg(uint8_t total_ic,uint8_t config[][6], uint8_t LTCPin)
{
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
  wakeup_idle (LTCPin);                                //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
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
}

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
  for (uint8_t i = 0; i < tx_len; i++)
  {
    spi_write(tx_Data[i]);

  }

  for (uint8_t i = 0; i < rx_len; i++)
  {
    rx_data[i] = (uint8_t)spi_read(0xFF);
  }

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