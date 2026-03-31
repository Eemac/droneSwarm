/* SENEX VR CONFIDENTIAL
* 
*  [2021] Senex VR, LLC 
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

/* Comment Syntax:
* -------------------------------------------------------------------------------------------
* |   Title   |   Meaning																	|
* -------------------------------------------------------------------------------------------
* |   @name   |   The name of the function being defined.									|
* |   @brief  |   A quick definition of what the function does.								|
* |   @param  |   A parameter in the function, and a simple description of what it is.		|
* |   @type   |   Whether the function is used in the CONTROLLER, the CORE, or BOTH.		|
* |   @note   |   An additional piece of information, usually when the function was tested.	|
* |   @return |   Possible values the fucntion returns, if any.								|
* -------------------------------------------------------------------------------------------
*/

#ifndef _SENEX_IMU_6_H
#define _SENEX_IMU_6_H

	#include "Arduino.h"
	#include "Math.h"
	#include <EEPROM.h>
	#include "SPI.h"

	struct ICM42688_BASE {
		//Which physical pin to use for SPI CS on hands
		unsigned char CSPin;

		uint8_t lastBank;

		uint32_t rawAccel[3];
		uint32_t rawGyro[3];

		double outputAccel[3];
		double outputGyro[3];

		double gyroOffset[3] = {0};
		double accelOffset[3] = {0};
	};


	/////////////////////////////////////////////////////////////////////////////////////////////////
	//										       INIT				 							   //
	/////////////////////////////////////////////////////////////////////////////////////////////////
	bool imu6begin(ICM42688_BASE &chip, uint8_t CSPin);

	bool imu6cal(ICM42688_BASE &chip);

	/////////////////////////////////////////////////////////////////////////////////////////////////
	//										       DATA				 							   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

	bool imu6readAccelGyro(ICM42688_BASE &chip, double *acceleration, double *rotation);

	bool imu6AccelCalibration(ICM42688_BASE &chip);
	bool imu6GyroCalibration(ICM42688_BASE &chip);

	bool imu6setCalibration(ICM42688_BASE &chip);

	void imu6printAccel(ICM42688_BASE &chip);
	void imu6printGyro(ICM42688_BASE &chip);


	/////////////////////////////////////////////////////////////////////////////////////////////////
	//										     HARDWARE			 							   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

	bool imu6read(ICM42688_BASE &chip, uint8_t reg, uint8_t len, unsigned char *data);

	bool imu6write(ICM42688_BASE &chip, uint8_t reg, uint8_t len, const unsigned char *data);

	bool imu6changeBank(ICM42688_BASE &chip, uint8_t bank);

#endif