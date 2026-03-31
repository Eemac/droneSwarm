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

#ifndef _SENEX_DRONE_CPP
#define _SENEX_DRONE_CPP

	//Basic Environment Functionality
	#include "Arduino.h"
	#include "Senex_Drone.h"

	#include "Wire.h"
	#include <SPI.h>

    #include <esp_task_wdt.h>

	#include <Senex_UWB.h>
	#include <Preferences.h>
	#include "Senex_IMU.h"

	#ifdef WIFI
		#include <Senex_Node_WiFi.h>
	#endif

	/////////////////////////////////////////////////////////////////////////////////////////////////
	//											  IO INIT										   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

	Preferences preferences;

	N_IO d_io;

	ICM20948_BASE icm20948;

	//Device Mode
	uint8_t _currentMode = 0xFF;

	//Low Power for LEDs
	bool _ledsEnabled = true;
	
	//Configures running mode for LED driver
	uint8_t _ledMode = 0;
	
	//Base Effect color
	uint8_t _color[4] = {0};

	//Secondary Effect Color
	uint8_t _color2[4] = {0};

	//LED effect period (ms)
	uint32_t _updatePeriod = 500UL;
	uint32_t _perMil = 0UL;
	uint32_t _perMilOld = 0UL;
	uint32_t _auxTimer = 0UL;
	uint32_t _perMilBounce = 0UL;

	//Array (24 * 4 bytes) for all color/brightness information
	uint8_t _pixelData[4] = {0};

	//Buffer Array for crossfade
	uint8_t _fPixelData[3] = {0};

	float battVoltage = 0.0;
	uint32_t batteryTimer = 0UL;

	uint8_t buffer = 0;


	//Wrapper for the photonics core
	void DroneIO::begin()
	{
		Serial.begin(250000);
		Serial.setTimeout(10);

		//Init I2C bus for the LED controller
		Wire.begin();
		//400kHz I2C speed
		Wire.setClock(400000);

		//Start SPI core
		SPI.begin();
		
		//Init SPI bus for the imus, barometer, and UWB transciever
		SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

		initDIOStruct(d_io);

		//ICM20948
		pinMode(NINE_AXIS_CS, OUTPUT);
		digitalWrite(NINE_AXIS_CS, HIGH);
		pinMode(NINE_AXIS_INT, INPUT);

		//ICM42688
		pinMode(SIX_AXIS_CS, OUTPUT);
		digitalWrite(SIX_AXIS_CS, HIGH);
		pinMode(SIX_AXIS_INT, INPUT);

		//Barometer
		pinMode(BARO_CS, OUTPUT);
		digitalWrite(BARO_CS, HIGH);

		//DWM3000
		pinMode(DWM_CS, OUTPUT);
		digitalWrite(DWM_CS, HIGH);
		pinMode(DWM_INT, INPUT);

		//Propellers
		pinMode(PWM_FR, OUTPUT);
		pinMode(PWM_FL, OUTPUT);
		pinMode(PWM_BR, OUTPUT);
		pinMode(PWM_BL, OUTPUT);
		digitalWrite(PWM_FR, LOW);
		digitalWrite(PWM_FL, LOW);
		digitalWrite(PWM_BR, LOW);
		digitalWrite(PWM_BL, LOW);

		//Voltage Sense
		pinMode(BATT_SENSE, INPUT);

		//Init ICM20948
		//Initialize the struct for the specific chip, at the correct position in the CORE array 
		Start(icm20948, 1);

		//If the chip did init, it is ready; if not, set the error flag
		if(setSensor(icm20948, false)) {Serial.println("[INFO]:  9-Axis Initialization:...................FAILED");}
		else {Serial.println("[INFO]:  9-Axis Initialization:...................PASSED");}

		//Init ICM42688
		Serial.println("INFO:  Setting up 6-Axis IMU...");

		//Init DWM3000
		// if(initUWB(drone)) {Serial.println("[INFO]:  UWB Transciever Initialization:..........PASSED");}
        // else{Serial.println("[INFO]:  UWB Transciever Initialization...........FAILED");}

		//Init LEDs
		if(initLEDs()) {Serial.println("[INFO]:  LED Initialization:......................PASSED");}
        else{Serial.println("[INFO]:  LED Initialization.......................FAILED");}

		Serial.println("INFO:  Flight Controller Hardware Init Complete");


		#ifdef WIFI
			initWifi(d_io);
		#endif
	}

	void DroneIO::initDIOStruct(N_IO &d_io) {
		
		//Load EEPROM DEV_ID
		preferences.begin("senex_node", false);
		uint32_t node_id = preferences.getUInt("nodeID", 0);
		d_io.chipID[0] = (uint8_t)(node_id >> 24);
		d_io.chipID[1] = (uint8_t)(node_id >> 16);
		d_io.chipID[2] = (uint8_t)(node_id >>  8);
		d_io.chipID[3] = (uint8_t)(node_id >>  0);
  		preferences.end();

  		//Begin telemetery 
		Serial.print("INFO:  Starting Drone at address: ");
		Serial.println((char)chipID[0] + (char)chipID[1] + (char)chipID[2] + (char)chipID[3]);


		d_io.uwbPacketNumber = 0UL;

		d_io.WiFiConnected = false;
		d_io.wifiTimeout = 0UL;

		d_io.serverTimeoutTimer = 0UL;
		d_io.update60HzTimer = 0UL;
		d_io.connectedToServer = false;

		d_io.UWBRunning = false;
		d_io.firmwareUpdate = false;

		d_io.mode = MODE_WIFI_SEARCH;
	}

	long timertimer = 0;
	long timertimerQ = 0;
	void DroneIO::update() {
		if(timertimer + 5000 < esp_timer_get_time()) {
			updateBattery();
			updateLEDs();
			updateFSM();
			timertimer = esp_timer_get_time();
		}

		if(timertimerQ + 1000000 < esp_timer_get_time()) {
			// Serial.print(d_io.totalTX);
            // Serial.print(" ");
            // Serial.println(d_io.RXTXmissmatch);
            timertimerQ = esp_timer_get_time();
		}

		#ifdef DEBUG
			doDebug(d_io);
		#endif








		unsigned long calibrationDelay = 0UL;
		//Poll hardware for Quat9 Data
		dmp_get_fifo(icm20948, out_data, true);
		Serial.print(out_data[0], HEX);
		Serial.print(" ");
		Serial.print(out_data[1], HEX);
		Serial.print(" ");
		Serial.println(out_data[2], HEX);


		//If available, get DMP biases for storage
		#ifdef ICM20948_CALIBRATION
			if(millis() - calibrationDelay > 1000)
			{
				//False bool for debug printouts
				getDMPBiases(icm20948, true);
				calibrationDelay = millis();
			}
		#endif

		//Main Loop debug delay
		#ifdef MAINLOOP_DEBUG_DELAY
			delay(MAINLOOP_DEBUG_DELAY_TIME * 8);
		#endif
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	//											   LEDs 										   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

		void DroneIO::setLedMode(uint8_t mode) {
			_ledMode = mode;
		}

		void DroneIO::setColor(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0, uint8_t brightnessScaler = 255) {
			_color[0] = red;
			_color[1] = green;
			_color[2] = blue;
			_color[3] = brightnessScaler;
		}

		void DroneIO::setColor2(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0, uint8_t brightnessScaler = 255) {
			_color2[0] = red;
			_color2[1] = green;
			_color2[2] = blue;
			_color2[3] = brightnessScaler;
		}

		void DroneIO::updateLEDs() {
			if(_ledsEnabled) {
				switch(d_io.mode) {
					case MODE_WIFI_SEARCH:
						setLedMode(LED_MODE_BLINK);
						setColor(255,200,0,30);
						setColor2(255,200,0,150);
						setUpdateRate(5);
						break;
					case MODE_SENEX_SEARCH:
						setLedMode(LED_MODE_FADE);
						setColor(255,200,0,30);
						setColor2(255,200,0,150);
						break;
					case MODE_IDLE:
						setLedMode(LED_MODE_FADE);
						setColor(0,255,0,50);
						setColor2(0,255,0,150);
						setUpdateRate(5);
						break;
					case MODE_RUN:
						setLedMode(LED_MODE_FADE);
						setColor(20,20,150,90);
						setColor2(20,20,150,150);
						setUpdateRate(5);
						break;
					case MODE_FIRMWARE_UPDATE:
						setLedMode(LED_MODE_SOLID);
						setColor(150,0,150,120);
						break;
					case MODE_ERR:
						setLedMode(LED_MODE_BLINK);
						setColor(150,0,0,90);
						setColor2(150,0,0,150);
						setUpdateRate(5);
						break;
					default:
						setLedMode(LED_MODE_OFF);
						break;
				}	
			}
			else {
				setLedMode(LED_MODE_OFF);
				_currentMode = 0xFF;
			}
			handlePixelColorUpdates();
		}

		//Push an update to WS2812B RGB LEDs
		void DroneIO::ledSetSolidColor(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0, uint8_t brightnessScaler = 255) {

			//Go through all of the pixels and update their values based on what was in the input packet
			_pixelData[0] = red;
			_pixelData[1] = green;
			_pixelData[2] = blue;
			_pixelData[3] = brightnessScaler;
		}

		void DroneIO::handlePixelColorUpdates() {
			//Update rate timer (0 to 100 * 10) for animations
			_perMil = (millis() % _updatePeriod) * 1000 / _updatePeriod;
			_perMilBounce = (uint32_t)(abs(int(_perMil) - 500) * 2);

			switch(_ledMode){
				case LED_MODE_OFF:
					ledSetSolidColor();

					_widgetInit = false;
					break;

				case LED_MODE_SOLID:
					ledSetSolidColor(_color[0], _color[1], _color[2], _color[3]);

					_widgetInit = false;
					break;

				case LED_MODE_FADE:
					ledSetSolidColor((_color[0] * _perMilBounce + _color2[0] * (1000 - _perMilBounce)) / 1000,
									 (_color[1] * _perMilBounce + _color2[1] * (1000 - _perMilBounce)) / 1000,
									 (_color[2] * _perMilBounce + _color2[2] * (1000 - _perMilBounce)) / 1000,
									 (_color[3] * _perMilBounce + _color2[3] * (1000 - _perMilBounce)) / 1000);

					_widgetInit = false;
					break;

				case LED_MODE_BLINK:
					if(_perMil % 1000 > 500) {
						ledSetSolidColor(_color[0], _color[1], _color[2], _color[3]);
					}
					else {
						ledSetSolidColor(_color2[0], _color2[1], _color2[2], _color2[3]);
					}

					_widgetInit = false;
					break;

				default:
					ledSetSolidColor();
					break;
			}

			writeToHardware();
		}

		void DroneIO::writeToHardware() {
			//Go through all of the pixels and update their values based on what was in the input packet
			_fPixelData[0] += max(-FADE_MAX_VAL, min(FADE_MAX_VAL, (_pixelData[0] * _pixelData[3] / 255) - _fPixelData[0]));
			_fPixelData[1] += max(-FADE_MAX_VAL, min(FADE_MAX_VAL, (_pixelData[1] * _pixelData[3] / 255) - _fPixelData[1]));
			_fPixelData[2] += max(-FADE_MAX_VAL, min(FADE_MAX_VAL, (_pixelData[2] * _pixelData[3] / 255) - _fPixelData[2]));
			//Red
			Wire.beginTransmission(0x6C);
			Wire.write(0x03);
			Wire.write((uint8_t)((uint16_t)_fPixelData[0] * 0x60 / 0xFF));
			Wire.endTransmission();

			//Blue
			Wire.beginTransmission(0x6C);
			Wire.write(0x04);
			Wire.write((uint8_t)((uint16_t)_fPixelData[1] * 0xF0 / 0xFF));
			Wire.endTransmission();

			//Green
			Wire.beginTransmission(0x6C);
			Wire.write(0x05);
			Wire.write((uint8_t)((uint16_t)_fPixelData[2] * 0xF0 / 0xFF));
			Wire.endTransmission();

			//Fade Rate
			uint8_t fade_rate = 0;
			Wire.beginTransmission(0x6C);
			Wire.write(0x02);
			Wire.write(0x80 | fade_rate & 0x07);
			Wire.endTransmission();
		}

		bool DroneIO::initDroneLEDs() {
			//Attempt to Read 
			uint8_t check = 0;
			//Init the I2C bus
			Wire.begin();

			Wire.beginTransmission(0x6C);
			Wire.write(0x00);
			Wire.endTransmission(false);
			Wire.requestFrom(0x6C, 1);
			while(Wire.available()){check = Wire.read();}       
			Wire.endTransmission();

			//WhoAmI should be A4 in hex
			if(check != 0xA4) {
				Serial.println(check, HEX);
				return true;
			}

			//Enable controller (normal mode)
			Wire.beginTransmission(0x6C);
			Wire.write(0x02);
			Wire.write(0x80);
			Wire.endTransmission();

			//Enable reading from current setting 0 (LEDA1 and LEDA2)
			Wire.beginTransmission(0x6C);
			Wire.write(0x09);
			Wire.write(0x88);
			Wire.endTransmission();

			//Enable reading from current setting 0 (LEDA3 and LEDA4)
			Wire.beginTransmission(0x6C);
			Wire.write(0x0A);
			Wire.write(0x88);
			Wire.endTransmission();

			//Enable reading from current setting 0 (LEDB1 and LEDB2)
			Wire.beginTransmission(0x6C);
			Wire.write(0x0B);
			Wire.write(0x88);
			Wire.endTransmission();

			//Enable reading from current setting 0 (LEDB3 and LEDB4)
			Wire.beginTransmission(0x6C);
			Wire.write(0x0C);
			Wire.write(0x88);
			Wire.endTransmission();
			return false;
	}

		void DroneIO::setUpdateRate(uint16_t dHz) {
			_updatePeriod = 10000 / dHz;
		}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	//											  BATTERY										   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

		float DroneIO::getVoltage() {
			return (float)analogRead(33) * 0.00116 + 0.3; //770k, 480k, 1.62x
		}

		void DroneIO::updateBattery() {
			if(millis() - batteryTimer > 10) {
				battVoltage = (float)(analogRead(BATT_SENSE) * 0.00115 + 0.14) * 0.1 + battVoltage * 0.9;
				// getChargingState();
				batteryTimer = millis();
			}
		}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	//											  CONTROL										   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

	void DroneIO::updateFSM() {

		//If the firmware is updating, show that
		if(d_io.firmwareUpdate == true) {
			d_io.mode = MODE_FIRMWARE_UPDATE;
		}
		else {
			//if the node has errored, keep it there until reset
			if(d_io.mode != MODE_ERR) {
				//Connected to WiFi?
				if(d_io.WiFiConnected == true) {
					//Connected to server?
					if(d_io.connectedToServer == true) {
						//UWB system Active
						if(d_io.UWBRunning == true) {
							d_io.mode = MODE_RUN;
						}
						else {
							d_io.mode = MODE_IDLE;
						}
					}

					//Otherwise looking for the server
					else {
						d_io.mode = MODE_SENEX_SEARCH;
					}
				}
				//
				else {
					d_io.mode = MODE_WIFI_SEARCH;
				}
			}
			else {
				d_io.mode = MODE_ERR;
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	//											   DEBUG										   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

	void DroneIO::doDebug(N_IO &d_io) {
		if(Serial.available() > 0) {
        	if(Serial.read() == 'A') {printDebugVars(d_io);}
        	else {printDWMReg();}
        }
	}
	void DroneIO::printDebugVars(N_IO &d_io) {
		if(Serial.available()) {
        	if(Serial.read() == 'A') {
				Serial.println("[DEBUG]: START Node Struct Variable Debug Printout");

				Serial.print("serverID:              0x"); Serial.print(d_io.serverID[0], HEX);
				Serial.print(" 0x"); Serial.print(d_io.serverID[1], HEX);
				Serial.print(" 0x"); Serial.print(d_io.serverID[2], HEX);
				Serial.print(" 0x"); Serial.println(d_io.serverID[3], HEX);

				Serial.print("ChipID:                0x"); Serial.print(d_io.chipID[0], HEX);
				Serial.print(" 0x"); Serial.print(d_io.chipID[1], HEX);
				Serial.print(" 0x"); Serial.print(d_io.chipID[2], HEX);
				Serial.print(" 0x"); Serial.println(d_io.chipID[3], HEX);

				Serial.print("uwbRecentID:              0x"); Serial.print(d_io.uwbRecentID[0], HEX);
				Serial.print(" 0x"); Serial.print(d_io.uwbRecentID[1], HEX);
				Serial.print(" 0x"); Serial.print(d_io.uwbRecentID[2], HEX);
				Serial.print(" 0x"); Serial.println(d_io.uwbRecentID[3], HEX);

				for(int i = 0; i < MAX_NODES; i++) {
					if(d_io.nodeAddresses[i * 4] + d_io.nodeAddresses[i * 4 + 1] + d_io.nodeAddresses[i * 4 + 2] + d_io.nodeAddresses[i * 4 + 3] != 0) {
						Serial.print("nodeAddresses position: "); Serial.print(i);
						Serial.print(" ---> 0x"); Serial.print(d_io.nodeAddresses[0], HEX);
						Serial.print(" 0x"); Serial.print(d_io.nodeAddresses[1], HEX);
						Serial.print(" 0x"); Serial.print(d_io.nodeAddresses[2], HEX);
						Serial.print(" 0x"); Serial.println(d_io.nodeAddresses[3], HEX);
					}
				}

				Serial.println("");

				Serial.print("systemTime:            "); Serial.println(d_io.systemTime);
				Serial.print("Mode: "); Serial.println(d_io.mode);

				Serial.print("WiFiConnected:         "); Serial.println(d_io.WiFiConnected);
				Serial.print("wifiTimeout:           "); Serial.println(d_io.wifiTimeout);
				Serial.print("serverTimeoutTimer:    "); Serial.println(d_io.serverTimeoutTimer);
				Serial.print("connectedToServer:     "); Serial.println(d_io.connectedToServer);
				Serial.print("update60HzTimer:       "); Serial.println(d_io.update60HzTimer);
				Serial.print("firmwareUpdate:        "); Serial.println(d_io.firmwareUpdate); Serial.println("");

				Serial.print("UWBRunning:            "); Serial.println(d_io.UWBRunning);
				Serial.print("nodeSettings:          "); Serial.println(d_io.nodeSettings);
				Serial.print("numNodeBases:          "); Serial.println(d_io.numNodeBases);
				Serial.print("numNodes:              "); Serial.println(d_io.numNodes);
				Serial.print("nodeListVersion:       "); Serial.println(d_io.nodeListVersion);
				Serial.print("packetNodeVersion:     "); Serial.println(d_io.packetNodeVersion); Serial.println("");

				Serial.print("rangeWindowPeriod:     "); Serial.println(d_io.rangeWindowPeriod);
				Serial.print("nodeSelfPosition:      "); Serial.println(d_io.nodeSelfPosition);
				Serial.print("uwbPacketNumber:       "); Serial.println(d_io.uwbPacketNumber);
				Serial.print("lastUWBPacketTimeUs:   "); Serial.println(d_io.lastUWBPacketTimeUs); Serial.println("");

				Serial.print("packetsUntilTx:        "); Serial.println(d_io.packetsUntilTx);
				Serial.print("usToTx:                "); Serial.println(d_io.usToTx);
				Serial.print("idleUWBBeaconTimer:    "); Serial.println(d_io.idleUWBBeaconTimer);
				Serial.print("lastRXTime:            "); Serial.println(d_io.lastRXTime);
				Serial.print("lastTXTime:            "); Serial.println(d_io.lastTXTime);
				Serial.print("packetStartTime:       "); Serial.println(d_io.packetStartTime); Serial.println("");

				

				Serial.println("[DEBUG]: END Node Struct Variable Debug Printout");
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	//												  MISC										   //
	/////////////////////////////////////////////////////////////////////////////////////////////////

	uint32_t bytesToUInt32_t(uint8_t *buffer) {
	    return (uint32_t)(buffer[3]) << 24 | (uint32_t)(buffer[2]) << 16 | (uint32_t)(buffer[1]) << 8 | (uint32_t)buffer[0];
	}

	void uint32_tToBytes(uint32_t in, uint8_t *buffer) {
	    buffer[0] = (uint8_t)(in >>  0);
	    buffer[1] = (uint8_t)(in >>  8);
	    buffer[2] = (uint8_t)(in >> 16);
	    buffer[3] = (uint8_t)(in >> 24);
	}

#endif