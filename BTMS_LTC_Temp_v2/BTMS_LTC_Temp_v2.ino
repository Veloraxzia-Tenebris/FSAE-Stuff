#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include <SPI.h>
#include "BTMS_LTC_Temp_v2.h"

// Upper / Lower pin
#define UPPER_LOWER 5

double temperatures[10];
int pins[5] = {0, 1, 2, 6, 4};

void setup() {
	pinMode(UPPER_LOWER, OUTPUT);
	for(int k = 0; k < 5; k++) {
		pinMode(pins[k], OUTPUT);
	}
	Serial.begin(9600);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV32);
	for(int k = 0; k < 5; k++) {
		digitalWrite(UPPER_LOWER, LOW);
		delay(10);
		LTCSetup(pins[k]);
		delay(10);
		digitalWrite(UPPER_LOWER, HIGH);
		delay(10);
		LTCSetup(pins[k]);
		delay(10);
	}
	delay(1000);
}

void loop() {
	for(int k = 0; k < 5; k++) {
		digitalWrite(UPPER_LOWER, LOW);
		delay(10);
		temperatures[k] = LTCLoop(pins[k]);
		delay(10);
		digitalWrite(UPPER_LOWER, HIGH);
		delay(10);
		temperatures[k + 1] = LTCLoop(pins[k]);
		delay(10);
	}

	Serial.println();
	for(int k = 0; k < 10; k++) {
		Serial.print(temperatures[k]);
		Serial.print(", ");
	}
	Serial.println();
}