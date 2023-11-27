/*******************************************************************************
 *
 *  File:         bsf_gps_tracker.h
 *
 *  Function:     Board Support File for Custom GPS.
 *
 *  Copyright:    Copyright (c) 2021 Leonel Lopes Parente
 *
 *  License:      MIT License. See accompanying LICENSE file.
 *
 *  Author:       Leonel Lopes Parente
 *
 *  Description:  This board has onboard USB (provided by onboard USB to serial).
 *                It supports automatic firmware upload and serial over USB.
 *                Has onboard display.
 *
 *                The standard I2C pins defined in the BSP do not match the
 *                GPIO pins that the display is connected to. Therefore the
 *                the I2C Wire object is explicitly initialized with the
 *                correct pins (see boardInit() below).
 *
 *                Schematic diagram and and pinout diagram show no onboard
 *                user programmable LED while LED_BUILTIN is defined in BSP.
 *                Definition in BSP is incorrect.
 *
 *                OLED_RST and LORA_RST are defined in BSP but neither is connected to GPIO.
 *                Definitions in BSP are incorrect.
 *
 *                CONNECTIONS AND PIN DEFINITIONS:
 *
 *                Indentifiers between parentheses are defined in the board's
 *                Board Support Package (BSP) which is part of the Arduino core.
 *
 *                Leds                GPIO
 *                ----                ----
 *                LED                 -            Incorrectly defined in BSP as LED_BUILTIN (2).
 *
 *                SPI/LoRa            GPIO
 *                ---                 ----
 *                MOSI  <――――――――――>  D10  (MOSI) (LORA_MOSI)
 *                MISO  <――――――――――>  D9  (MISO) (LORA_MISO)
 *                SCK   <――――――――――>  D8  (SCK)  (LORA_SCK)
 *                NSS   <――――――――――>  D5  (SS)   (LORA_CS)
 *                RST   <――――――――――>  D4         (LORA_RST)
 *                DIO0  <――――――――――>  D0         (LORA_IRQ)
 *                DIO1  <――――――――――>  D1
 *                DIO2  <――――――――――>  D2
 *
 *                Button switches     GPIO
 *                ------              ----
 *                Button <―――――――――>  36  (V_SP) Active-low
 *
 *                Battery measure     GPIO
 *                -------             ----
 *                VBAT  <――――――――――>  35  Battery voltage via 50% voltage divider
 *
 *  Docs:         https://docs.platformio.org/en/latest/boards/espressif32/ttgo-lora32-v1.html
 *
 *  Identifiers:  LMIC-node
 *                    board:         ttgo_lora32_v1
 *                PlatformIO
 *                    board:         ttgo-lora32-v1
 *                    platform:      espressif32
 *                Arduino
 *                    board:         ARDUINO_TTGO_LoRa32_V1
 *                    architecture:  ARDUINO_ARCH_ESP32
 *
 ******************************************************************************/

#pragma once

#ifndef BSF_GPS_TRACKER_H_
#define BSF_GPS_TRACKER_H_

#include "LMIC-node.h"
#include <TinyGPSPlus.h>

#define BATTERY_PIN 39
#define BATTERY_FACTOR 9.9

#define BATTERY_100_V 4.20
#define BATTERY_90_V 3.95
#define BATTERY_80_V 3.85
#define BATTERY_70_V 3.78
#define BATTERY_60_V 3.70
#define BATTERY_50_V 3.68
#define BATTERY_40_V 3.63
#define BATTERY_30_V 3.58
#define BATTERY_20_V 3.50
#define BATTERY_10_V 3.40
#define BATTERY_0_V 3.00


#define DEVICEID_DEFAULT "gpstracker" // Default deviceid value

// Wait for Serial
// Can be useful for boards with MCU with integrated USB support.
// #define WAITFOR_SERIAL_SECONDS_DEFAULT 10   // -1 waits indefinitely

// LMIC Clock Error
// This is only needed for slower 8-bit MCUs (e.g. 8MHz ATmega328 and ATmega32u4).
// Value is defined in parts per million (of MAX_CLOCK_ERROR).
// #ifndef LMIC_CLOCK_ERROR_PPM
//     #define LMIC_CLOCK_ERROR_PPM 0
// #endif

// Pin mappings for LoRa tranceiver
const lmic_pinmap lmic_pins = {
    .nss = D5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = D4, // See remark about LORA_RST above.
    .dio = {/*dio0*/ D0, /*dio1*/ D1, /*dio2*/ D2}
#ifdef MCCI_LMIC
    ,
    .rxtx_rx_active = 0,
    .rssi_cal = 10,
    .spi_freq = 8000000 /* 8 MHz */
#endif
};

#ifdef USE_SERIAL
#define serial Serial
#endif

#ifdef USE_LED
EasyLed led(LED_BUILTIN, EasyLed::ActiveLevel::Low);
#endif

// The TinyGPSPlus object
TinyGPSPlus gps;

void wakeGPS()
{
    digitalWrite(D3, HIGH);
    delay(100);
    Serial2.println("$PMTK101*32");
}

void sleepGPS()
{
    digitalWrite(D3, LOW);
    Serial2.println("$PMTK225,4*2F");
}

void hardwareLoop()
{
    if (Serial2.available())
    {
        int data = Serial2.read();
        //Serial.write(data);
        gps.encode(data);
    }
}

float GetBatteryPercent(float voltage)
{
    if (voltage >= BATTERY_100_V)
    {
        return 100.0;
    }
    else if (voltage >= BATTERY_90_V)
    {
        return (voltage - BATTERY_90_V) / (BATTERY_100_V - BATTERY_90_V) * 10.0 + 90;
    }
    else if (voltage >= BATTERY_80_V)
    {
        return (voltage - BATTERY_80_V) / (BATTERY_90_V - BATTERY_80_V) * 10.0 + 80;
    }
    else if (voltage >= BATTERY_70_V)
    {
        return (voltage - BATTERY_70_V) / (BATTERY_80_V - BATTERY_70_V) * 10.0 + 70;
    }
    else if (voltage >= BATTERY_60_V)
    {
        return (voltage - BATTERY_60_V) / (BATTERY_70_V - BATTERY_60_V) * 10.0 + 60;
    }
    else if (voltage >= BATTERY_50_V)
    {
        return (voltage - BATTERY_50_V) / (BATTERY_60_V - BATTERY_50_V) * 10.0 + 50;
    }
    else if (voltage >= BATTERY_40_V)
    {
        return (voltage - BATTERY_40_V) / (BATTERY_50_V - BATTERY_40_V) * 10.0 + 40;
    }
    else if (voltage >= BATTERY_30_V)
    {
        return (voltage - BATTERY_30_V) / (BATTERY_40_V - BATTERY_30_V) * 10.0 + 30;
    }
    else if (voltage >= BATTERY_20_V)
    {
        return (voltage - BATTERY_20_V) / (BATTERY_30_V - BATTERY_20_V) * 10.0 + 20;
    }
    else if (voltage >= BATTERY_10_V)
    {
        return (voltage - BATTERY_10_V) / (BATTERY_20_V - BATTERY_10_V) * 10.0 + 10;
    }
    else if (voltage >= BATTERY_0_V)
    {
        return (voltage - BATTERY_0_V) / (BATTERY_10_V - BATTERY_0_V) * 10.0;
    }
    else
    {
        return 0;
    }
}

float GetBatteryVoltage()
{
    float bat_read = analogRead(BATTERY_PIN) / 4096.0 * BATTERY_FACTOR;
    return bat_read;
}

bool boardInit(InitType initType)
{
    // This function is used to perform board specific initializations.
    // Required as part of standard template.

    // InitType::Hardware        Must be called at start of setup() before anything else.
    // InitType::PostInitSerial  Must be called after initSerial() before other initializations.

    bool success = true;
    switch (initType)
    {
    case InitType::Hardware:
        pinMode(D3, OUTPUT);
        Serial2.begin(9600, SERIAL_8N1, D7, D6);
        wakeGPS();
        // Note: Serial port and display are not yet initialized and cannot be used use here.
        break;

    case InitType::PostInitSerial:
        // Note: If enabled Serial port and display are already initialized here.
        // No actions required for this board.
        break;
    }
    return success;
}

#endif // BSF_TTGO_LORA32_V1_H_