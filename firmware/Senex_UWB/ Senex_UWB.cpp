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

#ifndef _SENEX_UWB_CPP
#define _SENEX_UWB_CPP

    #include "Arduino.h"
    #include <Senex_Node_IO.h>
    #include "Senex_UWB.h"
    #include <SPI.h>

    #include <esp_task_wdt.h>

    

    TaskHandle_t uwb_task;
    
    //Holy hell 10MHz!
    SPISettings dwm3000SpiSettings(10000000, MSBFIRST, SPI_MODE0);

    void initUWB(N_IO &n_io) {
        SPI.begin();

        pinMode(CS_PIN, OUTPUT);
        pinMode(RESET_PIN, OUTPUT);

        if(configUWB(n_io)) {
            n_io.mode = MODE_ERR;
        }

        Serial.println("[INFO]:  Launching UWB Communications Interface");
        
        xTaskCreatePinnedToCore(
            uwbTask, /* Function to implement the task */
            "UWB_Task", /* Name of the task */
            10000,  /* Stack size in words */
            (void*)&n_io,  /* Task input parameter */
            25,  /* Priority of the task */
            &uwb_task,  /* Task handle. */
            1); /* Core where the task should run */
        esp_task_wdt_add(uwb_task);

        //Just listen (unless the first node in the sequence)
        n_io.packetStartTime = 0xFFFFFFFF;
    }

    bool configUWB(N_IO &n_io) {
        //General purpose buffer
        uint8_t buffer[4] = {0,0,0,0};

        ///////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////  HARDWARE INIT  /////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        // Parameters:
        // * 850 kb/s, 
        // * PAC Size 8
        // * 16 symbol SFD



        //Disable DWM3000 SPI interface
        digitalWrite(CS_PIN, HIGH);

        //Reset chip to default
        digitalWrite(RESET_PIN, LOW);
        delay(2);
        pinMode(RESET_PIN, INPUT);

        //Allow for RC_INIT to RC_IDLE state transition 
        delay(25);
        Serial.println();
        Serial.println("--------------------------------------------------------");
        Serial.println("[INFO]:  Starting UWB Device Driver...");

        ///////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////  DWM INIT START  ////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

        //read system status register
        dwm_read(0x00, 0x44, 4, buffer);
        if(buffer[2] & 0x80) {Serial.println("[INFO]:  INIT_SPI_RDY:............................PASSED");}
        else{Serial.println("[INFO]:  INIT_SPI_RDY:............................FAILED"); return true;}

        //Test if the chip correctly reads (is the chip WhoAmI reading 0xDECA0302)
        dwm_read(0x00, 0x00, 4, buffer);
        if(bytesToUInt32_t(buffer) == 0xDECA0302) {Serial.println("[INFO]:  INIT_WHO_AM_I:...........................PASSED");}
        else{Serial.println("[INFO]:  INIT_WHO_AM_I:...........................FAILED"); return true;}

        //Read and load LDO (0x04low/0x05high) and Bias tune (0x0A) addresses, then use the kick function to move them to the dwm's register bank
        uint32_t bias_tune = (dwm_otpRead(0x0A) >> 16) & 0x1FU;
        if(dwm_otpRead(0x04) != 0 && dwm_otpRead(0x05) != 0 && bias_tune != 0) {
            dwm_andOr(0x0B, 0x08, 0xFFFFFFFF, 0x00000180, TWO_BYTE_AND_OR);
            dwm_andOr(0x11, 0x1F, 0xFFFFFFE0, bias_tune, TWO_BYTE_AND_OR);
        }
        else{Serial.println("[INFO]:  INIT_BIAS_OTP:...........................FAILED"); return true;}

        //Read the calibration values for temp and voltage readings
        dwmSettings.refTemp = (uint8_t)dwm_otpRead(0x09);
        dwmSettings.refVoltage = (uint8_t)dwm_otpRead(0x08);

        //Read and set the crystal time offset (xtrim)
        buffer[0] = (uint8_t)dwm_otpRead(0x1E) & 0x7f;
        dwm_write(0x09, 0x14, 1, buffer);

        if(dwmSettings.refTemp != 0 && dwmSettings.refVoltage != 0) {Serial.println("[INFO]:  INIT_BIAS_OTP:...........................PASSED");}
        else{Serial.println("[INFO]:  INIT_BIAS_OTP:...........................FAILED"); return true;}


        ///////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////  DWM INIT CONFIG  ////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

        //clear the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
        //then set the relevant bits according to configuration of the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode
        //Set PHR mode to standard packet length (PHR_MODE = 0)
        //Set PHR datarate to 850Kb/S (PHR_6M8 = 0)
        //Disable STS (CP_SPC = 0)
        //Disable Super Deterministic Code (SDC) mode (CP_SDC = 0). This may give better performance for ToA so may be enabled in the future
        //Set PDoA mode to off (DMW3000 doesn't support it) (PHR_MODE = 0)
        //SET 
        //SYS_CFG
        dwm_andOr(0x00, 0x10, 0xFFFC4FCF, 0x00000400, FOUR_BYTE_AND_OR);

        //Configure OPS tables for non-SCP mode (basically kick a set of parameters that we select)
        //Setting bit 10 kicks the operating param set
        //Setting bit 12 selects the 'short' operating param set
        //SYS_OTP
        dwm_andOr(0x0B, 0x08, 0xFFFFE7FF, 0x00001400, FOUR_BYTE_AND_OR);

        //Digital reciever configuration
        //DTUNE0 (SFD timeout) -> 129 symbols (128 -16 + 16 + 1)                    //0x00810000
        //Sets Preamble Acquisition Chunk size to 16 in the DTUNE0_ID registers     //0x00000001
        //Clear dtuneb4                                                           - //0x00000010
        //Keep previous vars                                                        //0x0000100C
        dwm_andOr(0x06, 0x00, 0x0000FFEC, 0x0081100D, FOUR_BYTE_AND_OR);

        //Sets the STS length to ~64uS, but this doesnt matter b/c we disable STS later
        //STS_CFG0_ID
        buffer[0] = 0x07;
        dwm_write(0x02, 0x00, 1, buffer);

        //clear the setting in the FINE_PLEN register.
        buffer[0] = 0x00;
        dwm_write(0x00, 0x29, 1, buffer);

        //configure default preamble detection threshold for other modes
        //DTUNE3
        buffer[0] = 0xCC;
        buffer[1] = 0x35;
        buffer[2] = 0x5F;
        buffer[3] = 0xAF;
        dwm_write(0x06, 0x0C, 4, buffer);

        /////////////////////////////////////////////////////////////////////////
        //Set the RX and TX preable code to 9
        //set the rx/tx frequency to ch5
        //set the SFD to ----+-+--++--+00 (deca-defined 16sym SFD)
        //CHAN_CTRL
        dwm_andOr(0x01, 0x14, 0xFFFFE000, 0x0000094C, FOUR_BYTE_AND_OR);

        /////////////////////////////////////////////////////////////////////////
        //TX_FCTRL
        //BYTE0: Set the send TX length to 14 bytes (12+2) (data+CRC)
        //BYTE1: Set TX Preamble Size (128 @ bits 12-15) (0x50), Set the TX Data Rate (0x04=6.8Mb/s, 0x00 = 850kb/s), Set the ranging bit (0x08)
        //BYTE3-5: FINE_PLEN and TXB_OFFSET are 0.
        buffer[0] = UWB_PACKET_LENGTH + 2; //(0x0E)
        buffer[1] = 0x58; //5C for 6.8mbps
        buffer[2] = 0x00;
        buffer[3] = 0x00;
        dwm_write(0x00, 0x24, 4, buffer);

        // dwm_andOr(0x00, 0x24, 0xFFFF0BFF, 0x00005C00 | UWB_PACKET_LENGTH + 2, FOUR_BYTE_AND_OR);
        // dwm_andOr(0x00, 0x24, 0xFFFFF400, 0x800, FOUR_BYTE_AND_OR);

        ///////////////////////////////// RF ////////////////////////////////////
        // Setup TX analog for ch5 (0x1C071134UL)
        // TX_CTRL_HI_ID
        buffer[0] = 0x34;
        buffer[1] = 0x11;
        buffer[2] = 0x07;
        buffer[3] = 0x1C;
        dwm_write(0x07, 0x1C, 4, buffer);

        //0x1F3C - sets PLL config values for channel 5 (0x1F3C) ch9 is (0x0F3C)
        //PLL_CFG_ID
        buffer[0] = 0x3C;
        buffer[1] = 0x1F;
        dwm_write(0x09, 0x00, 2, buffer);

        //LDO tuning register for tranciever (should be set to 0x14, no idea why...)
        //LDO_RLOAD_ID
        buffer[0] = 0x14;
        dwm_write(0x07, 0x51, 1, buffer);

        //Transmitter control register (no idea what this does, thanks datasheet)
        //TX_CTRL_LO_ID
        buffer[0] = 0x0E;
        dwm_write(0x07, 0x1A, 1, buffer);

        //Extend PLL lock delay for better fix (pll cal config)
        //PLL_CAL_ID
        buffer[0] = 0x81;
        dwm_write(0x09, 0x08, 1, buffer);

        //Verify PLL lock bit is cleared by writing a 1 (will clear this bit, and yes this is weird)
        //We will be able to verify if the PLL locks after calibration
        //SYS_STATUS_ID
        buffer[0] = 0x02;
        buffer[1] = 0x00;
        buffer[2] = 0x00;
        buffer[3] = 0x00;
        dwm_write(0x00, 0x44, 4, buffer);

        /////////////////////////////////////////////////////////////////////////
        //Set all clocks to AUTO mode - if PLL is ready, they will use PLL, otherwise use FAST_RC 
        buffer[0] = 0x00;
        buffer[1] = 0x02;
        dwm_write(0x11, 0x04, 2, buffer);

        //Set the AINIT2IDLE bit to calibrate the PLL and move into the IDLE_PLL state
        //Also go into IDLE state before IDLE_PLL transition
        dwm_andOr(0x11, 0x08, 0xFFFFFFFF, 0x00000100, TWO_BYTE_AND_OR);

        //Check to make sure PLL is locked (over 6 iterations, 20uS spacing), otherwise error
        bool pllLocked = false;
        for(uint8_t pllTries = 0; pllTries < 6; pllTries++) {
            delayMicroseconds(20);
            dwm_read(0x00, 0x44, 4, buffer);
            if(bytesToUInt32_t(buffer) & 0x00000002) {

                pllLocked = true;
                Serial.println("[INFO]:  PLL_CAL_LOCK:............................PASSED");
                break;
            }
        }
        if(!pllLocked) {Serial.println("[INFO]:  PLL_CAL_LOCK:............................FAILED"); return true;}

        //load RX Constant Look up Table (LUT)
        //Kick RX parameters from OTP into registers (ANDed bit 13 is set to 0 to allow for ch5 param set to be kicked)
        //OTP_CFG_ID, reg: DGC_KICK
        dwm_andOr(0x0B, 0x08, 0xFFFFDFFF, 0x00000040, TWO_BYTE_AND_OR);


        //64Mhz PRF tuning threshold
        //DGC_CFG
        dwm_andOr(0x03, 0x18, 0xFFFF81FF, 0x00006400, TWO_BYTE_AND_OR);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Probability generating function (PGF) calibration...
        //Critial calibration step, exit and re-run calibration if this fails

        //Save LDO values before calibration
        //LDO_CTRL_ID
        dwm_read(0x07, 0x48, 4, buffer);
        uint32_t ldoValues = bytesToUInt32_t(buffer);
        
        //Enable LDO_VDDMS1, LDO_VDDTX2, LDO_VDDIF2 LDO regulators for calibration step
        dwm_andOr(0x07, 0x48, 0xFFFFFFFF, 0x00000141, TWO_BYTE_AND_OR);

        //Set the RX calibration delay value to 0x02 and put the RX frontend into calibration mode (0x01)
        buffer[0] = 0x01;
        buffer[1] = 0x00;
        buffer[2] = 0x02;
        buffer[3] = 0x00;
        dwm_write(0x04, 0x0C, 4, buffer);

        //Fire the calibration sequence
        dwm_andOr(0x04, 0x0C, 0xFFFFFFFF, 0x00000010, ONE_BYTE_AND_OR);

        //TEST
        buffer[0] = 0x00;

        bool pgfCal = false;
        for(uint8_t pgfTries = 0; pgfTries < 3; pgfTries++) {
            delayMicroseconds(20);
            dwm_read(0x04, 0x20, 1, buffer);
            if(buffer[0] == 0x01) {
                pgfCal = true;
                Serial.println("[INFO]:  PGF_CALIBRATE:...........................PASSED");
                break;
            }
        }
        if(!pgfCal) {Serial.println("[INFO]:  PGF_CALIBRATE:...........................FAILED"); return true;}

        //Disable calibration and return to normal operating mode
        buffer[0] = 0x00;
        dwm_write(0x04, 0x0C, 1, buffer);

        //Clear the PGF calibration result
        buffer[0] = 0x01;
        dwm_write(0x04, 0x20, 1, buffer);

        //Set delay to 1? to read values from RX_CAL_RESI and RX_CAL_RESQ registers
        dwm_andOr(0x04, 0x0C, 0xFFFFFFFF, 0x00010000, FOUR_BYTE_AND_OR);

        //Verify RX_CAL_RESI
        dwm_read(0x04, 0x14, 4, buffer);
        if(bytesToUInt32_t(buffer) != 0x1FFFFFFF) {Serial.println("[INFO]:  PGF_CAL_RESI:............................PASSED");}
        else{Serial.println("[INFO]:  PGF_CAL_RESI:............................FAILED"); return true;}

        //Verify RX_CAL_RESQ
        dwm_read(0x04, 0x1C, 4, buffer);
        if(bytesToUInt32_t(buffer) != 0x1FFFFFFF) {Serial.println("[INFO]:  PGF_CAL_RESQ:............................PASSED");}
        else{Serial.println("[INFO]:  PGF_CAL_RESQ:............................FAILED"); return true;}

        //Return LDO Block to pre-calibration values
        //LDO_CTRL_ID
        uint32_tToBytes(ldoValues, buffer);
        dwm_write(0x07, 0x48, 4, buffer);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Configure TX PG_DELAY
        buffer[0] = 0x34;
        dwm_write(0x07, 0x1C, 1, buffer);

        // Configure TX power
        buffer[0] = 0xFE;
        buffer[1] = 0xFE;
        buffer[2] = 0xFE;
        buffer[3] = 0xFE;
        dwm_write(0x01, 0x0C, 4, buffer);

        //Set RX antenna delay CIA_CONF_ID
        dwmSettings.rx_ant_dly = 16385;
        buffer[0] = (uint8_t)dwmSettings.rx_ant_dly;
        buffer[1] = (uint8_t)(dwmSettings.rx_ant_dly >> 8);
        dwm_write(0x0E, 0x00, 2, buffer);

        //Set TX antenna delay TX_ANTD_ID
        dwmSettings.tx_ant_dly = 16385;
        buffer[0] = (uint8_t)dwmSettings.tx_ant_dly;
        buffer[1] = (uint8_t)(dwmSettings.tx_ant_dly >> 8);
        dwm_write(0x01, 0x04, 2, buffer);

        //Write node ID to TX buffer
        buffer[0] = n_io.chipID[0];
        buffer[1] = n_io.chipID[1];
        buffer[2] = n_io.chipID[2];
        buffer[3] = n_io.chipID[3];
        dwm_write(0x14, 0x00, 4, buffer);

        //DIAGNOSTICS
        dwm_andOr(0x0E, 0x00, 0xFFEFFFFF, 0x00000000, FOUR_BYTE_AND_OR);

        //Start listening...
        dwm_fastCommand(0x02);

        // //Delay between frames, in UWB microseconds.
        // #define POLL_TX_TO_RESP_RX_DLY_UUS 240
        // //Receive response timeout.
        // #define RESP_RX_TIMEOUT_UUS 400

        // dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        // dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);



        Serial.println("[INFO]:  UWB Device Driver Initialization Complete");
        Serial.println("--------------------------------------------------------");

        return false;
    }

    #define max(a,b) (((a) > (b)) ? (a) : (b))

        long counterA = 0;
    long counterB = 0;
    long counterC = 0;
    long counterPrev = 0;
    long timertimerB = 0;

    void uwbTask(void * io_in) {
        N_IO &n_io = *(N_IO*)(io_in);

        counterPrev = esp_timer_get_time();
        for(;;) {
            updateUWB(n_io);
        }
    }

    void updateUWB(N_IO &n_io) {
        counterPrev = n_io.systemTime;
        if(timertimerB + 4000000 < n_io.systemTime) {
            // Serial.println(counterA);
            // Serial.println(counterC);
            // counterA = 0;
            // counterC = 0;

            timertimerB = n_io.systemTime;
            esp_task_wdt_reset();
            // n_io.RXTXmissmatch = 0;
            n_io.totalTX = 0;
        }
        n_io.systemTime = esp_timer_get_time();

        //Connected to a large enough network and in sync with server?
        if(n_io.nodeListVersion >= n_io.packetNodeVersion &&
        n_io.numNodes > 1) {

            //If no other devices are sending TX for over UWB_PACKET_NO_RX_DURATION us
            if((n_io.systemTime - n_io.lastRXTime > UWB_PACKET_NO_RX_DURATION)) {
                //Send a beacon packet every UWB_PACKET_IDLE_LOOP_DURATION us. (defaults at 3s for both)
                if((n_io.systemTime + n_io.nodeSelfPosition * UWB_IDLE_OFFSET - n_io.idleUWBBeaconTimer > UWB_PACKET_IDLE_LOOP_DURATION)) {
                    sendUWBPacket(n_io);
                    n_io.idleUWBBeaconTimer = n_io.systemTime;
                }
            }

            //Otherwise, proceed with loop
            else {
                //Total Timebox:           |----- UWB TX REGION -----|
                //1st argument:            |^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ -> inf
                //2nd Argument:  -inf -> ^^^^^^^^^^^^^^^^^^^^| UwbSD |  
                // Check that the current time is after the start of this node's time slot OR this node is the next in line for transmit (if strict timeboxing disabled)
                if(n_io.packetStartTime < n_io.systemTime) {

                    //Check that the preposed start time is well enough in advance of the end of this send period minus the time it takes to send a packet
                    // n_io.packetStartTime + n_io.rangeWindowPeriod - UWB_PACKET_SEND_DURATION > n_io.systemTime) {
                    sendUWBPacket(n_io);
                    calculateRefireTime(n_io);
                }
            }
            
        }
        readUWBPacket(n_io);
    }

    void calculateRefireTime(N_IO &n_io) {
        //ON PACKET TX/RX acquisition!!!
        //Get number of RX sequences before TX
        for(uint16_t i = 0; i < n_io.numNodes; i++) {

            //Search for this node's id in array
            if(n_io.uwbRecentID[0] == n_io.nodeAddresses[4 * i + 0] && n_io.uwbRecentID[1] == n_io.nodeAddresses[4 * i + 1] && n_io.uwbRecentID[2] == n_io.nodeAddresses[4 * i + 2] && n_io.uwbRecentID[3] == n_io.nodeAddresses[4 * i + 3]) {
                //Most recent sender is ith position in the array

                //Number of packets that must be RXed until we send again
                n_io.packetsUntilTx = (uint16_t)((uint32_t)n_io.nodeSelfPosition + (uint32_t)n_io.numNodes - i - 1) % n_io.numNodes;

                //Time from most recent packet acquisition until TX auto-fire 
                n_io.packetStartTime = n_io.packetsUntilTx * n_io.rangeWindowPeriod;

                //If strict packet sending is enabled, force the node to wait an extra cycle minus TX time
                //assumes the cycle time isn't less than the fastest possible TX cycle 
                if(n_io.nodeSettings & 0x01) {
                    n_io.packetStartTime += max(n_io.rangeWindowPeriod - UWB_PACKET_SEND_DURATION, 0);
                }

                n_io.packetStartTime += esp_timer_get_time();

                return;
            }
        }
    }
    
    long omh = 0UL;
    bool sendUWBPacket(N_IO &n_io) {
        n_io.RXTXmissmatch++;
        n_io.totalTX++;
        long startCounter = ESP.getCycleCount();

        //General purpose buffer
        uint8_t buffer[4] = {0};

        //clear txDataSent flag
        buffer[0] = 0x80;
        dwm_write(0x00, 0x44, 1, buffer);

        ////////////////////////////////////////////////////////////////////////////
        //////////////////////////////  HEADER START  //////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        //Copy chipID to uwbRecentID (the most recent 'received' device is this one [this node just sent])
        n_io.uwbRecentID[0] = n_io.chipID[0];
        n_io.uwbRecentID[1] = n_io.chipID[1];
        n_io.uwbRecentID[2] = n_io.chipID[2];
        n_io.uwbRecentID[3] = n_io.chipID[3];

        //Write Node List Version to TX buffer
        uint32_tToBytes(n_io.nodeListVersion, buffer);

        //Write packet number to TX buffer
        uint32_tToBytes(n_io.uwbPacketNumber, buffer + 4);
        n_io.uwbPacketNumber++;

        //Add TX timestamp
        buffer[ 8] = n_io.uwbTxTimestamp[0];
        buffer[ 9] = n_io.uwbTxTimestamp[1];
        buffer[10] = n_io.uwbTxTimestamp[2];
        buffer[11] = n_io.uwbTxTimestamp[3];
        buffer[12] = n_io.uwbTxTimestamp[4];

        ////////////////////////////////////////////////////////////////////////////
        ////////////////////////////  BASE RANGES START  ///////////////////////////
        ////////////////////////////////////////////////////////////////////////////

        //Find first n_io.numNodeBases
        uint8_t basesAdded = 0;
        for(uint8_t base = 0; base < n_io.numNodes; base++) {
            //Node is a base
            if(n_io.nodeAddresses[base * 4 + 0] == 0x53 && n_io.nodeAddresses[base * 4 + 1] == 0x42) {
                //Copy Address of base station
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  0 + 13] = n_io.nodeAddresses[base * 4 + 0];
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  1 + 13] = n_io.nodeAddresses[base * 4 + 1];
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  2 + 13] = n_io.nodeAddresses[base * 4 + 2];
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  3 + 13] = n_io.nodeAddresses[base * 4 + 3];

				//Copy last recieved packet number
				buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  4 + 13] = n_io.baseNodePktCount[basesAdded * 4 + 0];
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  5 + 13] = n_io.baseNodePktCount[basesAdded * 4 + 1];
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  6 + 13] = n_io.baseNodePktCount[basesAdded * 4 + 2];
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  7 + 13] = n_io.baseNodePktCount[basesAdded * 4 + 3];
				
                //Copy last recieved timestamp
				buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  8 + 13] = n_io.baseNodeRxTime[basesAdded * 5 + 0];
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH +  9 + 13] = n_io.baseNodeRxTime[basesAdded * 5 + 1];
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH + 10 + 13] = n_io.baseNodeRxTime[basesAdded * 5 + 2];
                buffer[basesAdded * UWB_DATA_SNIP_LENGTH + 11 + 13] = n_io.baseNodeRxTime[basesAdded * 5 + 3];
				buffer[basesAdded * UWB_DATA_SNIP_LENGTH + 12 + 13] = n_io.baseNodeRxTime[basesAdded * 5 + 4];
            }

            //Stop looping if numNodes 
            if(basesAdded == n_io.numNodeBases) {
                break;
            }

        }

        // //BASE STATIONS ONLY
        // if(n_io.nodeSelfPosition < numNodeBases) {

        // }





        //Write to buffer with offset 4 so the TX ID is preserved
        dwm_write(0x14, 0x04, UWB_HEADER_LENGTH  - UWB_NODE_ID_LENGTH + UWB_DATA_SNIP_LENGTH * basesAdded, buffer);

        //Stop RX, start TX, once complete, begin RX
        dwm_fastCommand(0x00);
        dwm_fastCommand(0x0C);

        uint32_t bbb = 0;
        bbb = n_io.lastTXTime;

        //Update last time TX happened over UWB
        n_io.lastTXTime = esp_timer_get_time();
        bbb = n_io.lastTXTime - bbb;
        // if(bbb > 7400) {
            // Serial.println(bbb);
        // }


        dwm_read(0x00, 0x44, 1, buffer);
        while((buffer[0] & 0xF0) != 0xF0) {
            dwm_read(0x00, 0x44, 1, buffer);
        }

        //Read TX R-marker timestamp
        dwm_read(0x00, 0x74, 5, n_io.uwbTxTimestamp);


        //Write TX time to table
        // writeTimestamp(n_io.chipID, n_io.uwbPacketNumber, buffer);

        // startCounter = ESP.getCycleCount() - startCounter;
        // omh = max(omh, startCounter);

        // //Check reciever is in TX or has had an errored RX
        // uint8_t status[6] = {0};
        // dwm_read(0x00, 0x24, 6, status);

        //     for(int i = 0; i < 6; i++) {
        //         Serial.print(status[i], HEX);
        //         Serial.print(" ");
        //     }
        //     Serial.println(" end");

        // Serial.print("TX  ");
        // // Serial.println(esp_timer_get_time() / 100000);
        // Serial.println(omh);
        return false;
    }

    long omb = 0UL;

    bool readUWBPacket(N_IO &n_io) {
        uint8_t buffer[UWB_PACKET_LENGTH] = {0};
        uint32_t rxPacketVersion = 0;

        //Check reciever is in TX or has had an errored RX
        uint8_t status[6] = {0};
        dwm_read(0x00, 0x44, 6, status);

        //Listen unless actively sending (frame started [0x10], but not finished [0x80])
        if((status[0] & 0x90) != 0x10) {

            //Only read packet results and timestamp if packet is complete
            //and the transmission has been validated for completeness
            if(status[1] & 0x20) {
                dwm_read(0x12, 0x00, UWB_PACKET_LENGTH, buffer);

                //Start listening again
                dwm_fastCommand(0x02);
                
                //Set ChipID to be the node that just transmitted
                n_io.uwbRecentID[0] = buffer[0];
                n_io.uwbRecentID[1] = buffer[1];
                n_io.uwbRecentID[2] = buffer[2];
                n_io.uwbRecentID[3] = buffer[3];

                rxPacketVersion = bytesToUInt32_t(buffer + 4);

                //Update the most recent node's loop version 
                if(n_io.packetNodeVersion >= rxPacketVersion) {
                    n_io.packetNodeVersion = rxPacketVersion;
                }

                calculateRSSI();

                // for(int i = 0; i < UWB_PACKET_LENGTH; i++) {
                //     Serial.print(buffer[i], HEX);
                //     Serial.print(" ");
                // }
                // // Serial.println("");
                // Serial.println(esp_timer_get_time() / 100000);

                //Clear rxDataSent flag, await next send
                // buffer[0] = 0x00;
                // buffer[1] = 0x20;
                // dwm_write(0x00, 0x44, 2, buffer);

                calculateRefireTime(n_io);

                //Update last time TX happened over UWB
                n_io.lastRXTime = esp_timer_get_time();

                n_io.RXTXmissmatch--;
            }
        }

        return false;
    }

    void writeTimestamp(N_IO &n_io, uint32_t packetNumber, uint8_t timestamp[5]) {
        uint8_t buffer[4] = {0};
        uint8_t currentNodeNum = 0;

        //So sorry to whoever reads this if we ever have millions of devices running together, this is O(n) but its fast asf with 30 devices
        for(uint8_t i = 0; i < MAX_NODES; i++) {
            currentNodeNum = i;
            if(n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 0] == n_io.uwbRecentID[0]
            && n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 1] == n_io.uwbRecentID[1]
            && n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 2] == n_io.uwbRecentID[2]
            && n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 3] == n_io.uwbRecentID[3]) {
                for(uint16_t cpy = 0; cpy < 18; cpy++) {
                    n_io.tsArray[(i + 1) * TIMESTAMP_ROW_LENGTH - cpy - 1] = n_io.tsArray[(i + 1) * TIMESTAMP_ROW_LENGTH - TS_SAMPLE_LEN - cpy - 1];
                }
                //Prep array for new timestamp (move existing samples to the old sample slots)
                for(uint8_t cpy = 0; cpy < TS_SAMPLE_LEN; cpy++) {
                    n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + TS_SAMPLE_LEN * (TS_NUM_SAMPLES - 1) + TS_HEADER_LEN + cpy] = n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN  + TS_SAMPLE_LEN * (TS_NUM_SAMPLES - 2)+ cpy];
                }
                break;
            }

            //Haven't found device name in array, so add a new one if there's space, otherwise, ignore sample.
            else if(n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 0]
                  | n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 1]
                  | n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 2]
                  | n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 3] == 0) {
                //Copy new ID in
                n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 0] = n_io.uwbRecentID[0];
                n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 1] = n_io.uwbRecentID[1];
                n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 2] = n_io.uwbRecentID[2];
                n_io.tsArray[i * TIMESTAMP_ROW_LENGTH + 3] = n_io.uwbRecentID[3];
                break;
            }
        }

        //update ts in position
        uint32_tToBytes(packetNumber, buffer);
        //Update packet sequence ID
        n_io.tsArray[currentNodeNum * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN + 0] = buffer[0];
        n_io.tsArray[currentNodeNum * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN + 1] = buffer[1];
        n_io.tsArray[currentNodeNum * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN + 2] = buffer[2];
        n_io.tsArray[currentNodeNum * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN + 3] = buffer[3];

        n_io.tsArray[currentNodeNum * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN + 4] = timestamp[0];
        n_io.tsArray[currentNodeNum * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN + 5] = timestamp[1];
        n_io.tsArray[currentNodeNum * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN + 6] = timestamp[2];
        n_io.tsArray[currentNodeNum * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN + 7] = timestamp[3];
        n_io.tsArray[currentNodeNum * TIMESTAMP_ROW_LENGTH + TS_HEADER_LEN + 8] = timestamp[4];
    }

    float accum[10] = {0.0};
    uint16_t calculateRSSI() {
        uint8_t buffer[4] = {0};
        uint64_t c = 0;
        uint64_t n = 0;
        uint64_t d = 0;
        float rssi = 0.0;

        //Channel Impulse Response Power
        dwm_read(0x0C, 0x2C, 3, buffer);
        c = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16);

        //Preamble Accumulation Count
        dwm_read(0x0C, 0x58, 2, buffer);
        n = buffer[0] + ((buffer[1] & 0x0F) << 8);

        //DGC Decision
        dwm_read(0x03, 0x63, 1, buffer);
        d = (buffer[0] & 0x70) >> 4;

        rssi = 10 * log10(c * 0x200000 / n / n) + 6 * d - 121.7;

        //Moving average
        accum[4] = accum[3];
        accum[3] = accum[2];
        accum[2] = accum[1];
        accum[1] = accum[0];
        accum[0] = rssi;
        // Serial.println((accum[0] + accum[1] + accum[2] + accum[3] + accum[4]) / 5.0);
        Serial.println(rssi);

        return rssi;
    }

    // 
    void printDWMReg() {
        char string[30];
        char *subStrings[3];
        char *ptr = NULL;
        uint8_t index = 0;
        uint8_t buffer[4] = {0};
        
      
      if(Serial.available()) {
        String a = Serial.readString() + "+";// read the incoming data as string
        a.toCharArray(string, a.length());
        
        ptr = strtok(string, ",");  // delimiter
        while (ptr != NULL)
        {
          subStrings[index] = ptr;
          index++;
          ptr = strtok(NULL, ",");
        }

        //
        uint8_t bank = 0;
        uint8_t reg = 0;
        uint8_t len = 0;

        if(strchr(subStrings[0],'x') == NULL) {bank = atoi(subStrings[0]);} else{bank = strtoul(subStrings[0], NULL, 16);}
        if(strchr(subStrings[1],'x') == NULL) {reg = atoi(subStrings[1]);} else{reg = strtoul(subStrings[1], NULL, 16);}
        if(strchr(subStrings[2],'x') == NULL) {len = atoi(subStrings[2]);} else{len = strtoul(subStrings[2], NULL, 16);}

        Serial.print("Bank: 0x"); Serial.print(bank, HEX);
        Serial.print(", Reg: 0x"); Serial.print(reg, HEX);
        Serial.print(", Data: ");

        dwm_read(bank, reg, len, buffer);

        for(int i = atoi(subStrings[2]); i > 0; i--) {
            Serial.print(buffer[i - 1], HEX);
            Serial.print(" ");
        }
        Serial.println();
      }
    }

    void dwm_fastCommand(uint8_t command) {
        uint8_t header;

        //Format command (write,FAST,command(0-31), 1)
        header = 0x81 | ((command & 0x1F) << 1);

        //Perform serial read
        SPI.beginTransaction(dwm3000SpiSettings);
        digitalWrite(CS_PIN, LOW);

        //Header send
        SPI.transfer(header);
        //End SPI command
        SPI.endTransaction();
        digitalWrite(CS_PIN, HIGH);
    }
    // Working as of 8/9/24
    void dwm_read(uint8_t regFileID, uint8_t indx, uint16_t length, uint8_t *buffer) {
        uint8_t header[2];           // Buffer to compose header in

        //Format high (left) byte first
        header[0] = 0x40 | ((regFileID & 0x1F) << 1) | indx >> 6;

        //Low (right) bit second
        header[1] = (indx & 0x3F) << 2;


        //Perform serial read
        SPI.beginTransaction(dwm3000SpiSettings);
        digitalWrite(CS_PIN, LOW);

        //Header send
        SPI.transfer(header[0]);
        SPI.transfer(header[1]);

        //Read SPI values
        for(int i = 0; i < length; i++) {
            buffer[i] = SPI.transfer(JUNK);
        }

        //End SPI command
        SPI.endTransaction();
        digitalWrite(CS_PIN, HIGH);
    }
    // Working as of 8/9/24
    void dwm_write(uint8_t regFileID, uint8_t indx, uint16_t length, uint8_t *buffer) {
        uint8_t  header[2];           // Buffer to compose header in

        //Format high (left) byte first
        header[0] = 0xC0 | (regFileID & 0x1F) << 1 | indx >> 6;

        //Low (right) bit second
        header[1] = (indx & 0x3F) << 2;


        //Perform serial read
        SPI.beginTransaction(dwm3000SpiSettings);
        digitalWrite(CS_PIN, LOW);

        //Header send
        SPI.transfer(header[0]);
        SPI.transfer(header[1]);

        //Write SPI values
        for(int i = 0; i < length; i++) {
           SPI.transfer(buffer[i]);
        }

        //End SPI command
        SPI.endTransaction();
        digitalWrite(CS_PIN, HIGH);
    }

    //length 1,2,4 (one two, four byte and/or would look like this: 0x000000UU, 0x000000UU for 1 byte, 0x0000UUUU, 0x0000UUUU for 2 byte, 0xUUUUUUUU, 0xUUUUUUUU for 4 byte)
    void dwm_andOr(uint8_t regFileID, uint8_t indx, uint32_t andStr, uint32_t orStr, uint8_t byteWidth) {
        uint8_t  header[2];           // Buffer to compose header in
        uint8_t buffer[8];

        uint8_t length = 1 << byteWidth;

        //Format high (left) byte first
        //long write, reg bank, reg index top bit
        header[0] = 0xC0 | (regFileID & 0x1F) << 1 | indx >> 6;

        //Low (right) byte second
        //low part of the register index
        header[1] = (indx & 0x3F) << 2 | byteWidth;

        //Format bytestring to 
        for(int i = 0; i < length / 2; i++) {
            buffer[i] = (uint8_t)(andStr >> (8 * i));
            buffer[length / 2 + i] = (uint8_t)(orStr >> (8 * i));
        }

        //Perform serial read
        SPI.beginTransaction(dwm3000SpiSettings);
        digitalWrite(CS_PIN, LOW);

        //Header send
        SPI.transfer(header[0]);
        SPI.transfer(header[1]);

        //Write SPI values
        for(int i = 0; i < length; i++) {
           SPI.transfer(buffer[i]);
        }

        //End SPI command
        SPI.endTransaction();
        digitalWrite(CS_PIN, HIGH);
    }

    // Working as of 8/9/24
    uint32_t dwm_otpRead(uint16_t address) {
        uint8_t tempReg[2] = {0x00, 0x00};
        uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
        //Using bank 0x0B, the OTP bank

        //Set the OTP_MAN bit in the OTP_SF register.
        //This enables manual control over the OTP interface
        tempReg[0] = 0x01;
        dwm_write(0x0B, 0x08, 1, tempReg);

        //Set the address to access in OTP memory using the OTP_ADDR register.
        tempReg[0] = (uint8_t)(address);
        tempReg[1] = (uint8_t)(address >> 8);
        dwm_write(0x0B, 0x04, 2, tempReg);

        //Set the OTP_READ bit and clear the OTP_MAN bit.
        //This transfers functinality back to the OTP engine and fires a read at the OTP_ADDR address
        //The output will become visable in the OTP_RDATA register
        tempReg[0] = 0x02;
        dwm_write(0x0B, 0x08, 1, tempReg);

        //Read 32 bits from the OTP_RDATA register and output the four bytes as a uint32_t
        dwm_read(0x0B, 0x10, 4, data);
        return bytesToUInt32_t(data);
    }

    void voltTempHelper() {
        uint8_t buffer[4];
        uint8_t ldo_ctrl[1];
        uint8_t sar_status[1];
        bool reading_rdy = false;

        //Enable LDO for temp sense
        dwm_read(0x07, 0x48, 1, ldo_ctrl);
        dwm_andOr(0x07, 0x48, 0xFFFFFFFF, 0x00000002, ONE_BYTE_AND_OR);

        //Enable Tsense function
        buffer[0] = 0x04;
        dwm_write(0x07, 0x34, 1, buffer);

        //read all SAR inputs
        buffer[0] = 0x01;
        dwm_write(0x08, 0x00, 1, buffer);

        //Wait until reading has completed
        while (!reading_rdy) {
            //Poll SAR_STATUS register
            dwm_read(0x08, 0x04, 1, sar_status);
            if(sar_status[0] == 0x01) {
                reading_rdy = true;
            }
            delayMicroseconds(1);
        }

        //Read values from the SAR_READING_ID register (top 8 bits are temperature, low 8 are voltage)
        dwm_read(0x08, 0x08, 2, buffer);

        //set outputs
        dwmSettings.currVoltage = buffer[0];
        dwmSettings.currTemp = buffer[1];

        //Clear SAR enable
        buffer[0] = 0x00;
        dwm_write(0x08, 0x00, 1, buffer);


        //Disable Tsense function
        buffer[0] = 0x00;
        dwm_write(0x07, 0x34, 1, buffer);

        //Restore previous LDO status
        dwm_write(0x07, 0x48, 1, ldo_ctrl);
    }

    float getTemp() {
        //Update temp/voltage readings
        voltTempHelper();
        // the User Manual formula is: Temperature (°C) = ( (SAR_LTEMP – OTP_READ(Vtemp @ 20°C) ) x 1.05)        // Vtemp @ 20°C
        return (float)((dwmSettings.currTemp - dwmSettings.refTemp) * 1.05f) + 20.0f;
    }

    float getVoltage() {
        //Update temp/voltage readings
        voltTempHelper();
        // Bench measurements gives approximately: VDDBAT = sar_read * Vref / max_code * 16x_atten   - assume Vref @ 3.0V
        return (float)((float)(dwmSettings.currVoltage - dwmSettings.refVoltage) * 0.4f * 16 / 255) + 3.0f;
    }

#endif


        // if(status[1]) {
        //     for(int i = 0; i < 6; i++) {
        //         Serial.print(status[i], HEX);
        //         Serial.print(" ");
        //     }
        //     Serial.println(" end");
        // }