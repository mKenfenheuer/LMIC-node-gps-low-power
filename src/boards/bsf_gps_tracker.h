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
 *                MISO  <――――――――――>  D9   (MISO) (LORA_MISO)
 *                SCK   <――――――――――>  D8   (SCK)  (LORA_SCK)
 *                NSS   <――――――――――>  D4   (SS)   (LORA_CS)
 *                RST   <――――――――――>  D5          (LORA_RST)
 *                DIO0  <――――――――――>  D2          (LORA_IRQ)
 *                DIO1  <――――――――――>  D1
 *                DIO2  <――――――――――>  D0
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

#define MTCK GPIO_NUM_39
#define MTDO GPIO_NUM_40
#define MTDI GPIO_NUM_41
#define MTMS GPIO_NUM_42

#ifndef BSF_GPS_TRACKER_H_
#define BSF_GPS_TRACKER_H_

#define DEVICEID_DEFAULT "gpstracker" // Default deviceid value

#include "LMIC-node.h"
#include <TinyGPSPlus.h>

#define BATTERY_RT 220
#define BATTERY_RB 100

#define BATTERY_PIN D3
#define BATTERY_FACTOR 1.0f / ((float)BATTERY_RB / ((float)BATTERY_RT + (float)BATTERY_RB)) * 3.3f

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
extern const lmic_pinmap lmic_pins;

#ifdef USE_SERIAL
#define serial Serial
#endif

#ifdef USE_LED
extern EasyLed led;
#endif

// The TinyGPSPlus object
extern TinyGPSPlus gps;

void wakeGPS();

void sleepGPS();

void hardwareLoop();

float GetBatteryPercent(float voltage);

float GetBatteryVoltage();

bool boardInit(InitType initType);

#endif // BSF_TTGO_LORA32_V1_H_