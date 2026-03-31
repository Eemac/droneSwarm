/* SENEX VR CONFIDENTIAL
* 
*  [2020-2025] Senex VR, LLC 
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

#ifndef _SENEX_DRONE_H
#define _SENEX_DRONE_H


	class DroneIO
	{
		public:

			/////////////////////////////////////////////////////////////////////////////////////////////////
			//											  IO INIT										   //
			/////////////////////////////////////////////////////////////////////////////////////////////////
			DroneIO(void){};

			/*
			* @name:	NodeIO::begin
			* @brief:	Start digital LED driver + assign/setup pins
			* @return:	void							
			*/
			void begin();
			// void droneHWInit();

			void initDIOStruct(N_IO &d_io);

			void writeEepromParams(N_IO &d_io);

			void update();

			bool initDroneLEDs();

			/////////////////////////////////////////////////////////////////////////////////////////////////
			//											  SETTERS										   //
			/////////////////////////////////////////////////////////////////////////////////////////////////
			/*
			* @name:	NodeIO::setMode
			* @brief:	Update the current LED mode with a mode code.
			* @param:	uint8_t _mode 					== LED mode
			* @return:	void
			*/
			void setLedMode(uint8_t mode);

			void updateLEDs();

			/*
			* @name:	NodeIO::setUpdateRate
			* @brief:	Set how fast the IO unit should update (faster will be mor resource intensive).
			* @param:	uint8_t _mode 					== Core IMU struct
			* @return:	void
			*/
			void setUpdateRate(uint16_t dHz);

			/*
			* @name:	NodeIO::setColor
			* @brief:	Sets the base color (RGB + Brightness scaler [0-255]) for the hexagon
			* @param:	uint8_t red 					== 0-255 red brightness
			* @param:	uint8_t green 					== 0-255 green brightness
			* @param:	uint8_t blue 					== 0-255 blue brightness
			* @param:	uint8_t brightnessScaler 		== 0-255 brightness scaler
			* @return:	void
			*/
			void setColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightnessScaler);

			/*
			* @name:	NodeIO::setColor2
			* @brief:	Sets the secondary color (RGB + Brightness scaler [0-255]) for the hexagon
			* @param:	uint8_t red 					== 0-255 red brightness
			* @param:	uint8_t green 					== 0-255 green brightness
			* @param:	uint8_t blue 					== 0-255 blue brightness
			* @param:	uint8_t brightnessScaler 		== 0-255 brightness scaler
			* @return:	void
			*/
			void setColor2(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightnessScaler);
			

			/*
			* @name:	NodeIO::setColor2
			* @brief:	Sets the secondary color (RGB + Brightness scaler [0-255]) for the hexagon
			* @param:	uint8_t red 					== 0-255 red brightness
			* @param:	uint8_t green 					== 0-255 green brightness
			* @param:	uint8_t blue 					== 0-255 blue brightness
			* @param:	uint8_t brightnessScaler 		== 0-255 brightness scaler
			* @return:	void
			*/
			void ledSetSolidColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightnessScaler);


			/////////////////////////////////////////////////////////////////////////////////////////////////
			//											  GETTERS										   //
			/////////////////////////////////////////////////////////////////////////////////////////////////

			void updateBattery();
			float getVoltage();


			void updateFSM();

			void doDebug(N_IO &d_io);
			void printDebugVars(N_IO &d_io);
			

		/////////////////////////////////////////////////////////////////////////////////////////////////
		//											  INTERNAL										   //
		/////////////////////////////////////////////////////////////////////////////////////////////////
		private:
			/*
			* @name:	NodeIO::writeToHardware
			* @brief:	Perform hardware write from LED buffer to 24 LED nose cone array
			* @return:	void
			*/
			bool initLEDs();
			void setLEDs(uint8_t red, uint8_t blue, uint8_t green, uint8_t fade_rate);

	};

	uint32_t bytesToUInt32_t(uint8_t *buffer);
	void uint32_tToBytes(uint32_t in, uint8_t *buffer);	

#endif


