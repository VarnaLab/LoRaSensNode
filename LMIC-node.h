/*******************************************************************************
 *
 *  File:         LMIC-node.h
 * 
 *  Function:     LMIC-node main header file.
 * 
 *  Copyright:    Copyright (c) 2021 Leonel Lopes Parente
 *                Portions Copyright (c) 2018 Terry Moore, MCCI
 *
 *                Permission is hereby granted, free of charge, to anyone 
 *                obtaining a copy of this document and accompanying files to do, 
 *                whatever they want with them without any restriction, including,
 *                but not limited to, copying, modification and redistribution.
 *                The above copyright notice and this permission notice shall be 
 *                included in all copies or substantial portions of the Software.
 * 
 *                THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 * 
 *  License:      MIT License. See accompanying LICENSE file.
 * 
 *  Author:       Leonel Lopes Parente
 *                 
 ******************************************************************************/

#pragma once

#ifndef LMIC_NODE_H_
#define LMIC_NODE_H_

#include <Arduino.h>
#include "lmic.h"
#include "hal/hal.h"

#ifdef USE_DISPLAY
    #include <Wire.h>
    #include "U8x8lib.h"
#endif

#ifdef USE_LED
    #include "EasyLed.h"
#endif

enum class InitType { Hardware, PostInitSerial };
enum class PrintTarget { All, Serial, Display };


#define DEVICEID_DEFAULT "ttgo-lora32-v1"  // Default deviceid value

// Wait for Serial
// Can be useful for boards with MCU with integrated USB support.
// #define WAITFOR_SERIAL_SECONDS_DEFAULT 10   // -1 waits indefinitely  

// LMIC Clock Error
// This is only needed for slower 8-bit MCUs (e.g. 8MHz ATmega328 and ATmega32u4).
// Value is defined in parts per million (of MAX_CLOCK_ERROR).
// #ifndef LMIC_CLOCK_ERROR_PPM
//     #define LMIC_CLOCK_ERROR_PPM 0
// #endif   

#define USE_SERIAL

#ifdef BLALBALUSE_SERIAL
    HardwareSerial& serial = Serial;
#endif  

#ifdef USE_LED
    #error Invalid option: USE_LED. This board has no onboard user LED.
    // EasyLed led(<external LED GPIO>, EasyLed::ActiveLevel::Low);
#endif

#ifdef USE_DISPLAY
    // Create U8x8 instance for SSD1306 OLED display (no reset) using hardware I2C.
    U8X8_SSD1306_128X64_NONAME_HW_I2C display(/*rst*/ U8X8_PIN_NONE, /*scl*/ 15, /*sda*/ 4);
#endif


const dr_t DefaultABPDataRate = DR_SF7;
const s1_t DefaultABPTxPower =  14;

// Global declarations
void setupLMIC();
void loopLMIC();

// Forward declarations
static void doWorkCallback(osjob_t* job);
void processWork(ostime_t timestamp);
void processDownlink(ostime_t eventTimestamp, uint8_t fPort, uint8_t* data, uint8_t dataLength);
void onLmicEvent(void *pUserData, ev_t ev);
void displayTxSymbol(bool visible);

#ifndef DO_WORK_INTERVAL_SECONDS            // Should be set in platformio.ini
    #define DO_WORK_INTERVAL_SECONDS 300    // Default 5 minutes if not set
#endif    

#define TIMESTAMP_WIDTH 12 // Number of columns to display eventtime (zero-padded)
#define MESSAGE_INDENT TIMESTAMP_WIDTH + 3

// Determine which LMIC library is used
#ifdef _LMIC_CONFIG_PRECONDITIONS_H_   
    #define MCCI_LMIC
#else
    #define CLASSIC_LMIC
#endif    

#if !defined(ABP_ACTIVATION) && !defined(OTAA_ACTIVATION)
    #define OTAA_ACTIVATION
#endif

enum class ActivationMode {OTAA, ABP};
#ifdef OTAA_ACTIVATION
    const ActivationMode activationMode = ActivationMode::OTAA;
#else    
    const ActivationMode activationMode = ActivationMode::ABP;
#endif    


#include "keyfiles/lorawan-keys.h"

    
#if defined(ABP_ACTIVATION) && defined(OTAA_ACTIVATION)
    #error Only one of ABP_ACTIVATION and OTAA_ACTIVATION can be defined.
#endif

#if defined(ABP_ACTIVATION) && defined(ABP_DEVICEID)
    const char deviceId[] = ABP_DEVICEID;
#elif defined(DEVICEID)
    const char deviceId[] = DEVICEID;
#else
    const char deviceId[] = DEVICEID_DEFAULT;
#endif

// Allow WAITFOR_SERIAL_SECONDS to be defined in platformio.ini.
// If used it shall be defined in the [common] section.
// The common setting will only be used for boards that have 
// WAITFOR_SERIAL_SECONDS_DEFAULT defined (in BSP) with a value != 0
#if defined(WAITFOR_SERIAL_SECONDS_DEFAULT)  && WAITFOR_SERIAL_SECONDS_DEFAULT != 0
    #ifdef WAITFOR_SERIAL_SECONDS
        #define WAITFOR_SERIAL_S WAITFOR_SERIAL_SECONDS
    #else
        #define WAITFOR_SERIAL_S WAITFOR_SERIAL_SECONDS_DEFAULT
    #endif
#else
    #define WAITFOR_SERIAL_S 0
#endif 

#if defined(ABP_ACTIVATION) && defined(CLASSIC_LMIC)
    #error Do NOT use ABP activation when using the deprecated IBM LMIC framework library. \
           On The Things Network V3 this will cause a downlink message for EVERY uplink message \
           because it does properly handle MAC commands. 
#endif

#ifdef OTAA_ACTIVATION
    #if !defined(OTAA_DEVEUI) || !defined(OTAA_APPEUI) || !defined(OTAA_APPKEY)
        #error One or more LoRaWAN keys (OTAA_DEVEUI, OTAA_APPEUI, OTAA_APPKEY) are not defined.
    #endif 
#else
    // ABP activation
    #if !defined(ABP_DEVADDR) || !defined(ABP_NWKSKEY) || !defined(ABP_APPSKEY)
        #error One or more LoRaWAN keys (ABP_DEVADDR, ABP_NWKSKEY, ABP_APPSKEY) are not defined.
    #endif
#endif

// Determine if a valid region is defined.
// This actually has little effect because
// CLASSIC LMIC: defines CFG_eu868 by default,
// MCCI LMIC: if no region is defined it
// sets CFG_eu868 as default.
#if ( \
    ( defined(CLASSIC_LMIC) \
      && !( defined(CFG_eu868) \
            || defined(CFG_us915) ) ) \
    || \
    ( defined(MCCI_LMIC) \
      && !( defined(CFG_as923) \
            || defined(CFG_as923jp) \
            || defined(CFG_au915) \
            || defined(CFG_eu868) \
            || defined(CFG_in866) \
            || defined(CFG_kr920) \
            || defined(CFG_us915) ) ) \
)
    #Error No valid LoRaWAN region defined
#endif   

#ifndef MCCI_LMIC
    #define LMIC_ERROR_SUCCESS 0
    typedef int lmic_tx_error_t;

    // In MCCI LMIC these are already defined.
    // This macro can be used to initalize an array of event strings
    #define LEGACY_LMIC_EVENT_NAME_TABLE__INIT \
                "<<zero>>", \
                "EV_SCAN_TIMEOUT", "EV_BEACON_FOUND", \
                "EV_BEACON_MISSED", "EV_BEACON_TRACKED", "EV_JOINING", \
                "EV_JOINED", "EV_RFU1", "EV_JOIN_FAILED", "EV_REJOIN_FAILED", \
                "EV_TXCOMPLETE", "EV_LOST_TSYNC", "EV_RESET", \
                "EV_RXCOMPLETE", "EV_LINK_DEAD", "EV_LINK_ALIVE"

    // If working on an AVR (or worried about memory size), you can use this multi-zero
    // string and put this in a single const F() string to store it in program memory.
    // Index through this counting up from 0, until you get to the entry you want or 
    // to an entry that begins with a \0.
    #define LEGACY_LMIC_EVENT_NAME_MULTISZ__INIT \
                "<<zero>>\0" \                                                           \
                "EV_SCAN_TIMEOUT\0" "EV_BEACON_FOUND\0" \
                "EV_BEACON_MISSED\0" "EV_BEACON_TRACKED\0" "EV_JOINING\0" \
                "EV_JOINED\0" "EV_RFU1\0" "EV_JOIN_FAILED\0" "EV_REJOIN_FAILED\0" \
                "EV_TXCOMPLETE\0" "EV_LOST_TSYNC\0" "EV_RESET\0" \
                "EV_RXCOMPLETE\0" "EV_LINK_DEAD\0" "EV_LINK_ALIVE\0"   
#endif // LMIC_MCCI   

#ifdef USE_DISPLAY 
    uint8_t transmitSymbol[8] = {0x18, 0x18, 0x00, 0x24, 0x99, 0x42, 0x3c, 0x00}; 
    #define ROW_0             0
    #define ROW_1             1
    #define ROW_2             2
    #define ROW_3             3
    #define ROW_4             4
    #define ROW_5             5
    #define ROW_6             6
    #define ROW_7             7    
    #define HEADER_ROW        ROW_0
    #define DEVICEID_ROW      ROW_1
    #define INTERVAL_ROW      ROW_2
    #define TIME_ROW          ROW_4
    #define EVENT_ROW         ROW_5
    #define STATUS_ROW        ROW_6
    #define FRMCNTRS_ROW      ROW_7
    #define COL_0             0
    #define ABPMODE_COL       10
    #define CLMICSYMBOL_COL   14
    #define TXSYMBOL_COL      15

    void initDisplay()
    {
        display.begin();
        display.setFont(u8x8_font_victoriamedium8_r); 
    }

    void displayTxSymbol(bool visible = true)
    {
        if (visible)
        {
            display.drawTile(TXSYMBOL_COL, ROW_0, 1, transmitSymbol);
        }
        else
        {
            display.drawGlyph(TXSYMBOL_COL, ROW_0, char(0x20));
        }
    }    
#endif // USE_DISPLAY



#endif  // LMIC_NODE_H_
