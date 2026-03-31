/* SENEX VR CONFIDENTIAL
* 
*  [2021-2025] Senex VR, LLC 
*  All Rights Reserved.
* 
* NOTICE:  All information contained herein is, and remains
* the property of Senex VR, LLC and its suppliers,
* if any. The intellectual and technical concepts contained
* herein are proprietary to Senex VR, LLC and its suppliers and may 
* be covered by U.S. and Foreign Patents, patents in process, and 
* are protected by trade secret or copyright law. Dissemination of 
* this information or reproduction of this material is strictly 
* forbidden unless prior written permission is obtained
* from Senex VR, LLC.
*/

#ifndef _SENEX_IMU_6_CPP
#define _SENEX_IMU_6_CPP

	#include "Senex_IMU_6.h"
	#include "Arduino.h"
	#include <Preferences.h>

	Preferences imu6pref;

	/////////////////////////////////////////////////////////////////////////////////////////////////
	//										       INIT				 							   //
	/////////////////////////////////////////////////////////////////////////////////////////////////
	bool imu6begin(ICM42688_BASE &chip, uint8_t CSPin) {
		bool check = false;

		chip.CSPin = CSPin;
		chip.lastBank = 0xFF;

		unsigned char regChar[4] = {0x00};

		//Reset the chip
		regChar[0] = 0x01;
		imu6changeBank(chip, 0x00);
		check = imu6write(chip, 0x00, 1, regChar);

		//Wait for the chip to come back up
		delay(50);

		//WhoAmI address check after reset 
		imu6read(chip, 0x75, 1, &regChar[0]);
		if(regChar[0] != 0x47) {Serial.println(regChar[0]); return true;}

		//Set accelerometer to 8g scale, 1kHz output
		regChar[0] = 0x26; //46 for 4g, 26 for 8g
		check |= imu6write(chip, 0x50, 1, regChar);

		//Set gyro to 2000 dps, 1kHz output
		regChar[0] = 0x06; //46 for 4g, 26 for 8g
		check |= imu6write(chip, 0x4F, 1, regChar);



		//Set UI filter to order 2 for gyro, decimator to 3rd order, LPF BW to 170hz
		regChar[0] = 0x36; //46 for 4g, 26 for 8g
		check |= imu6write(chip, 0x51, 1, regChar);
		regChar[0] = 0x75; //accel, gyro
		check |= imu6write(chip, 0x52, 1, regChar);

		regChar[0] = 0x06; //46 for 4g, 26 for 8g
		check |= imu6write(chip, 0x4F, 1, regChar);

		//set IIR filter BW to ODR/4 for LN, 16x averaging for GLP, 16x averaging for ALP

		//Emable gyro and accel
		regChar[0] = 0x0F; //46 for 4g, 26 for 8g
		check |= imu6write(chip, 0x4E, 1, regChar);

		check |= imu6setCalibration(chip);

		return check;
	}

	bool imu6cal(ICM42688_BASE &chip) {
		bool check = false;
		return check;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	//										       DATA				 							   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

	bool imu6readAccelGyro(ICM42688_BASE &chip, double *acceleration, double *rotation) {
		bool check = false;

		unsigned char regChar[2] = {0};

		imu6read(chip, 0x1F, 0x02, regChar);
		acceleration[0] = ((double)(int16_t((regChar[0] << 8) + regChar[1])))/4096.0;

		imu6read(chip, 0x21, 0x02, regChar);
		acceleration[1] = ((double)(int16_t((regChar[0] << 8) + regChar[1])))/4096.0;

		imu6read(chip, 0x23, 0x02, regChar);
		acceleration[2] = ((double)(int16_t((regChar[0] << 8) + regChar[1])))/4096.0;



		imu6read(chip, 0x25, 0x02, regChar);
		rotation[0] = ((double)(int16_t((regChar[0] << 8) + regChar[1])))/16.4 - chip.gyroOffset[0];

		imu6read(chip, 0x27, 0x02, regChar);
		rotation[1] = ((double)(int16_t((regChar[0] << 8) + regChar[1])))/16.4 - chip.gyroOffset[1];

		imu6read(chip, 0x29, 0x02, regChar);
		rotation[2] = ((double)(int16_t((regChar[0] << 8) + regChar[1])))/16.4 - chip.gyroOffset[2];


		//Copy to internal buffer
		chip.outputAccel[0] = acceleration[0];
		chip.outputAccel[1] = acceleration[1];
		chip.outputAccel[2] = acceleration[2];

		chip.outputGyro[0] = rotation[0];
		chip.outputGyro[1] = rotation[1];
		chip.outputGyro[2] = rotation[2];

		return check;
	}

	bool imu6AccelCalibration(ICM42688_BASE &chip) {
		bool check = false;

		double acceleration[3] = {0.0};

		unsigned char regChar[2] = {0};

		for(long i = 0; i  < 1000; i++) {
			imu6read(chip, 0x1F, 0x02, regChar);
			acceleration[0] += ((double)(int16_t((regChar[0] << 8) + regChar[1])))/4096.0;

			imu6read(chip, 0x21, 0x02, regChar);
			acceleration[1] += ((double)(int16_t((regChar[0] << 8) + regChar[1])))/4096.0;

			imu6read(chip, 0x23, 0x02, regChar);
			acceleration[2] += ((double)(int16_t((regChar[0] << 8) + regChar[1])))/4096.0;

			delay(5);
		}

		acceleration[0] /= 1000;
		acceleration[1] /= 1000;
		acceleration[2] /= 1000;

		imu6pref.begin("senex_node", false);

		imu6pref.putFloat("accelX", acceleration[0]);
		imu6pref.putFloat("accelY", acceleration[1]);
		imu6pref.putFloat("accelZ", acceleration[2]);

		imu6pref.end();

		chip.accelOffset[0] = acceleration[0];
		chip.accelOffset[1] = acceleration[1];
		chip.accelOffset[2] = acceleration[2];

		Serial.println("Accelerometer Calibration Complete");

		return check;
	}

	bool imu6GyroCalibration(ICM42688_BASE &chip) {
		bool check = false;

		double rotation[3] = {0.0};

		unsigned char regChar[2] = {0};

		for(long i = 0; i  < 1000; i++) {
			imu6read(chip, 0x25, 0x02, regChar);
			rotation[0] += ((double)(int16_t((regChar[0] << 8) + regChar[1])))/16.4;

			imu6read(chip, 0x27, 0x02, regChar);
			rotation[1] += ((double)(int16_t((regChar[0] << 8) + regChar[1])))/16.4;

			imu6read(chip, 0x29, 0x02, regChar);
			rotation[2] += ((double)(int16_t((regChar[0] << 8) + regChar[1])))/16.4;

			delay(5);
		}

		rotation[0] /= 1000;
		rotation[1] /= 1000;
		rotation[2] /= 1000;

		imu6pref.begin("senex_node", false);

		imu6pref.putFloat("gyroX", rotation[0]);
		imu6pref.putFloat("gyroY", rotation[1]);
		imu6pref.putFloat("gyroZ", rotation[2]);

		imu6pref.end();

		chip.gyroOffset[0] = rotation[0];
		chip.gyroOffset[1] = rotation[1];
		chip.gyroOffset[2] = rotation[2];

		Serial.println("Gyroscope Calibration Complete");

		return check;
	}

	bool imu6setCalibration(ICM42688_BASE &chip) {
		imu6pref.begin("senex_node", false);

		Serial.println("Setting Calibration...");

		if(imu6pref.isKey("gyroX")) {
			chip.gyroOffset[0] = imu6pref.getFloat("gyroX", 0);
			chip.gyroOffset[1] = imu6pref.getFloat("gyroY", 0);
			chip.gyroOffset[2] = imu6pref.getFloat("gyroZ", 0);
		}

		if(imu6pref.isKey("accelX")) {
			chip.accelOffset[0] = imu6pref.getFloat("accelX", 0);
			chip.accelOffset[1] = imu6pref.getFloat("accelY", 0);
			chip.accelOffset[2] = imu6pref.getFloat("accelZ", 0);
		}
		
		imu6pref.end();

		return false;
	}


	/////////////////////////////////////////////////////////////////////////////////////////////////
	//										     HARDWARE			 							   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

	bool imu6read(ICM42688_BASE &chip, uint8_t reg, uint8_t len, unsigned char *data) {
		//Enable the new chip
		digitalWrite(chip.CSPin, LOW);

		//write start register 
		SPI.transfer(((reg & 0x7F) | 0x80));

		//Read data and tranfer to buffer char array
		for (unsigned char i = 0; i < len; i++)
		{
			data[i] = SPI.transfer(0x00);
		}

		//Disable the chip once read is done
		digitalWrite(chip.CSPin, HIGH);

		return false;
	}

	bool imu6write(ICM42688_BASE &chip, uint8_t reg, uint8_t len, const unsigned char *data) {
		unsigned char i = 0;
		//Enable the new chip
		digitalWrite(chip.CSPin, LOW);

		//Send the address of the start register
		SPI.transfer(((reg & 0x7F) | 0x00));

		for(i = 0; i < len; i++)
		{
			SPI.transfer(data[i]);
		}

		//Disable the chip once write is done
		digitalWrite(chip.CSPin, HIGH);

		return false;
	}

	bool imu6changeBank(ICM42688_BASE &chip, uint8_t bank) {
		bool check = false;

		//if the bank we're calling is the same one as we are already on, do nothing
		if(bank == chip.lastBank)
		{
			return check;
		}
		else
		{
			//set the last bank to the one we're about to set
			chip.lastBank = bank;
			bank = bank<<4;
			check = imu6write(chip, 0x76, 1, &bank);
		}

		return check;
	}

	void imu6printAccel(ICM42688_BASE &chip) {
		Serial.print("ACCEL DEBUG: ");
		Serial.print(chip.outputAccel[0]);
		Serial.print(",");
		Serial.print(chip.outputAccel[1]);
		Serial.print(",");
		Serial.println(chip.outputAccel[2]);
	}

	void imu6printGyro(ICM42688_BASE &chip) {
		Serial.print("GYRO DEBUG:  ");
		Serial.print(chip.outputGyro[0]);
		Serial.print(",");
		Serial.print(chip.outputGyro[1]);
		Serial.print(",");
		Serial.println(chip.outputGyro[2]);
	}

#endif