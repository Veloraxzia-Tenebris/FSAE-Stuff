#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include <SPI.h>
#include "SERCOM.h"
#include "BTMS_LTC_Temp_v2.h"

double temperatures[5];
int pins[5] = {0, 1, 2, 6, 4};

void setup() {
	for(int k = 0; k < 5; k++) {
		pinMode(pins[k], OUTPUT);
	}
	Serial.begin(9600);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV32);
	for(int k = 0; k < 5; k++) {
		LTCSetup(pins[k]);
	}
	delay(1000);
}

void loop() {
	for(int k = 0; k < 5; k++) {
		temperatures[k] = LTCLoop(pins[k]);
	}

	Serial.println();
	for(int k = 0; k < 5; k++) {
		Serial.print(temperatures[k]);
	}
	Serial.println();
}