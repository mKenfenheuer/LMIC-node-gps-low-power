#pragma once

#ifndef LMIC_USER_H_
#define LMIC_USER_H_

#include "WiFi.h"
#include "LMIC-node.h"

void UserSetup();
void UserLoop();
void UserPrepareMessage();
void UserDownlinkMessage(ostime_t txCompleteTimestamp, uint8_t fPort, uint8_t *data, uint8_t dataLength);
void UserRXTXComplete();

#endif