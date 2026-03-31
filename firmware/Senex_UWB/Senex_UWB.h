 /* SENEX VR CONFIDENTIAL
 * 
 *  [2020-2024] Senex VR, LLC 
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

#ifndef _SENEX_UWB_H
#define _SENEX_UWB_H


    #include "Senex_Node_IO.h"

    // no sub-address for register write
    #define NO_SUB                              0xFF

    #define WRITE                               0x80
    #define WRITE_SUB                           0xC0
    #define READ                                0x00
    #define READ_SUB                            0x40
    #define RW_SUB_EXT                          0x80

    #define OTP_CFG_ID                          0xB0008  
    #define OTP_ADDR_ID                         0xB0004 

    #define JUNK 0x00

    typedef enum {
        DW3000_SPI_RD_BIT    =                  0x0000U,
        DW3000_SPI_WR_BIT    =                  0x8000U,
        DW3000_SPI_AND_OR_8  =                  0x8001U,
        DW3000_SPI_AND_OR_16 =                  0x8002U,
        DW3000_SPI_AND_OR_32 =                  0x8003U,
    }spi_transaction_mode;

    #define ONE_BYTE_AND_OR                     1
    #define TWO_BYTE_AND_OR                     2
    #define FOUR_BYTE_AND_OR                    3

    #define CS_PIN                              5
    #define RESET_PIN                           4

    #define UWB_HEADER_LENGTH                  17
    #define UWB_DATA_SNIP_LENGTH               13


    #define UWB_PACKET_LENGTH                  12
    #define UWB_NODE_ID_LENGTH                  4
    #define UWB_PACKET_SEND_DURATION          538 //DWM3000 TX time in us
    #define UWB_IDLE_OFFSET                  5000
    #define UWB_PACKET_NO_RX_DURATION     3000000
    #define UWB_PACKET_IDLE_LOOP_DURATION 1000000

    struct {
        uint8_t refVoltage;
        uint8_t refTemp;
        uint8_t currVoltage;
        uint8_t currTemp;

        uint16_t rx_ant_dly;
        uint16_t tx_ant_dly;
    } dwmSettings;

    void uwbTask(void * parameter);

    void initUWB(N_IO &n_io);


    bool configUWB(N_IO &n_io);
    void updateUWB(N_IO &n_io);

    bool sendUWBPacket(N_IO &n_io);
    bool readUWBPacket(N_IO &n_io);

    uint16_t calculateRSSI();

    void calculateRefireTime(N_IO &n_io);
    //Low level SPI interactions 
    void dwm_read(uint8_t regFileID, uint8_t indx, uint16_t length, uint8_t *buffer);
    void dwm_write(uint8_t regFileID, uint8_t indx, uint16_t length, uint8_t *buffer);

    void dwm_andOr(uint8_t regFileID, uint8_t indx, uint32_t andStr, uint32_t orStr, uint8_t byteWidth);

    uint32_t dwm_otpRead(uint16_t address);

    void dwm_fastCommand(uint8_t command);

    void voltTempHelper();

    float getTemp();
    float getVoltage();

    void printDWMReg();
#endif 