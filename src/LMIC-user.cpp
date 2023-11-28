#include "LMIC-user.h"

typedef struct
{
    double lat;
    double lon;
} pos;

typedef enum
{
    STATE_INIT = 0,
    STATE_WAIT_GPS_FIRST_FIX = 1,
    STATE_WAIT_GPS_FIX = 2,
    STATE_GPS_MOVING = 3,
    STATE_GPS_STANDING = 4,
    STATE_SEND_NETWORK_MESSAGE = 5,
    STATE_DEEP_SLEEP_REQUESTED = 6
} SystemState;

RTC_DATA_ATTR int systemState = SystemState::STATE_INIT;
const float kmpdeg = 111.195;
float coslat;
RTC_DATA_ATTR pos lastSentPos;
RTC_DATA_ATTR bool hadGPSFix = false;
const uint8_t maxNetworkCount = 1;
const uint8_t payloadBufferLength = maxNetworkCount * 7;
uint8_t payloadBuffer[payloadBufferLength];
bool didSendNetworkMessage = false;

void SetState(SystemState newState)
{
    switch (newState)
    {
    case STATE_INIT:
        systemState = newState;
        printEvent("State switched to STATE_INIT");
        break;
    case STATE_WAIT_GPS_FIRST_FIX:
        systemState = newState;
        printEvent("State switched to STATE_WAIT_GPS_FIRST_FIX");
        break;
    case STATE_WAIT_GPS_FIX:
        systemState = newState;
        printEvent("State switched to STATE_WAIT_GPS_FIX");
        break;
    case STATE_GPS_MOVING:
        systemState = newState;
        printEvent("State switched to STATE_GPS_MOVING");
        break;
    case STATE_GPS_STANDING:
        systemState = newState;
        printEvent("State switched to STATE_GPS_STANDING");
        break;
    case STATE_SEND_NETWORK_MESSAGE:
        systemState = newState;
        printEvent("State switched to STATE_SEND_NETWORK_MESSAGE");
        break;
    case STATE_DEEP_SLEEP_REQUESTED:
        systemState = newState;
        printEvent("State switched to STATE_DEEP_SLEEP_REQUESTED");
        break;
    default:
        printEvent("Tried to set state to unknown state: " + String(systemState));
        break;
    }
}

float dist(const pos &a, const pos &b)
{
    if (coslat == 0.0)
    { // no need to calculate every time
        float lat = (a.lat + b.lat) / 2;
        coslat = cos(lat * PI / 180);
    }

    float dlat = a.lat - b.lat;
    float dlon = (a.lon - b.lon) * coslat;

    return sqrt(dlat * dlat + dlon * dlon) * kmpdeg * 1000;
}

bool IsLocationValid()
{
    if (!gps.location.isValid())
    {
        return false;
    }
    return true;
}

void GoDeepSleep()
{
    sleepGPS();
    saveSession();
    Serial.println("Go DeepSleep after " + String(millis()) + "ms.");
    Serial.flush();
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_SECONDS * 1000000);
    esp_deep_sleep_start();
}

void SendGPSMessage()
{
    if (msSinceLastMessage() < MESSAGE_MIN_DELAY && getMessageCount() > 0)
    {
        return;
    }

    if (systemState == STATE_WAIT_GPS_FIRST_FIX ||
        systemState == STATE_WAIT_GPS_FIX)
    {
        printEvent("Got GPS Fix.");
        SetState(STATE_GPS_STANDING);
    }

    ostime_t timestamp = os_getTime();
    uint8_t payloadLength = 0;
    uint8_t fPort = 0;
    fPort = 1;
    float lat, lon, alt, course, speed, hdop, sats;
    lat = gps.location.lat();
    lon = gps.location.lng();
    alt = gps.altitude.meters();
    course = gps.course.deg();
    speed = gps.speed.kmph();
    sats = gps.satellites.value();
    hdop = gps.hdop.hdop();
    lastSentPos = {lat, lon};

    hadGPSFix = true;

    unsigned char *puc;

    payloadLength = 0;
    puc = (unsigned char *)(&lat);
    payloadBuffer[payloadLength++] = puc[0];
    payloadBuffer[payloadLength++] = puc[1];
    payloadBuffer[payloadLength++] = puc[2];
    payloadBuffer[payloadLength++] = puc[3];
    puc = (unsigned char *)(&lon);
    payloadBuffer[payloadLength++] = puc[0];
    payloadBuffer[payloadLength++] = puc[1];
    payloadBuffer[payloadLength++] = puc[2];
    payloadBuffer[payloadLength++] = puc[3];
    puc = (unsigned char *)(&alt);
    payloadBuffer[payloadLength++] = puc[0];
    payloadBuffer[payloadLength++] = puc[1];
    payloadBuffer[payloadLength++] = puc[2];
    payloadBuffer[payloadLength++] = puc[3];
    puc = (unsigned char *)(&course);
    payloadBuffer[payloadLength++] = puc[0];
    payloadBuffer[payloadLength++] = puc[1];
    payloadBuffer[payloadLength++] = puc[2];
    payloadBuffer[payloadLength++] = puc[3];
    puc = (unsigned char *)(&speed);
    payloadBuffer[payloadLength++] = puc[0];
    payloadBuffer[payloadLength++] = puc[1];
    payloadBuffer[payloadLength++] = puc[2];
    payloadBuffer[payloadLength++] = puc[3];
    puc = (unsigned char *)(&hdop);
    payloadBuffer[payloadLength++] = puc[0];
    payloadBuffer[payloadLength++] = puc[1];
    payloadBuffer[payloadLength++] = puc[2];
    payloadBuffer[payloadLength++] = puc[3];

    printEvent("Message queued. Sending now. FPORT: " + String(fPort));
    scheduleMessage(fPort, payloadBuffer, payloadLength);
}

void SendNetworkMessage()
{
    if (msSinceLastMessage() < MESSAGE_MIN_DELAY && getMessageCount() > 0)
    {
        return;
    }

    ostime_t timestamp = os_getTime();
    uint8_t payloadLength = 0;
    uint8_t fPort = 0;
    SetState(STATE_DEEP_SLEEP_REQUESTED);

    if (!didSendNetworkMessage)
    {

        WiFi.setSleep(WIFI_PS_NONE);
        uint8_t num = WiFi.scanNetworks();
        if (num == 0)
        {
            payloadBuffer[0] = 0;
            fPort = 2;
        }
        else
        {
            printEvent("Found networks: " + String(num));

            fPort = 2;
            num = min(maxNetworkCount, num);
            payloadBuffer[0] = (uint8_t)num;
            payloadLength++;
            for (int i = 0; i < num; ++i)
            {
                memcpy(&payloadBuffer[payloadLength], WiFi.BSSID(i), 6);
                payloadLength += 6;
                payloadBuffer[payloadLength] = (uint8_t)(WiFi.RSSI(i) * -1);
                payloadLength++;
            }

            /*float batV = GetBatteryVoltage();
            float batP = GetBatteryPercent(batV);

            payloadBuffer[payloadLength] = ((batV - 3) / 2.0) * 255;
            payloadBuffer[payloadLength + 1] = (batP / 100.0) * 255;
            payloadLength += 2;*/
        }

        WiFi.setSleep(WIFI_PS_MAX_MODEM);
        printEvent("Message queued. Sending now. FPORT: " + String(fPort));
        scheduleMessage(fPort, payloadBuffer, payloadLength);
    }
    else if (getMessageQueue() <= 0 && getMessageCount() > 0)
    {
        printEvent("Forcing deep sleep now.");
        GoDeepSleep();
    }
}

void UserSetup()
{
    wakeGPS();
    SetState(STATE_INIT);
}

void UserLoop()
{
}

void UserPrepareMessage()
{
    switch (systemState)
    {
    case STATE_INIT:
        if (!hadGPSFix)
            SetState(STATE_WAIT_GPS_FIRST_FIX);
        else
            SetState(STATE_WAIT_GPS_FIX);
        break;
    case STATE_WAIT_GPS_FIRST_FIX:
        if (millis() > GPS_FIRST_FIX_TIMEOUT)
        {
            SetState(STATE_SEND_NETWORK_MESSAGE);
            printEvent("Sending network message as fallback");
            SendNetworkMessage();
        }
        else
        {
            if (IsLocationValid())
            {
                SendGPSMessage();
            }
        }
        break;
    case STATE_WAIT_GPS_FIX:
        if (millis() > GPS_FIX_TIMEOUT)
        {
            SetState(STATE_SEND_NETWORK_MESSAGE);
            printEvent("Sending network message as fallback");
            SendNetworkMessage();
        }
        else
        {
            if (IsLocationValid())
            {
                SendGPSMessage();
            }
        }
        break;
    case STATE_GPS_STANDING:
        if (hadGPSFix)
        {
            pos position = {gps.location.lat(), gps.location.lng()};
            float distance = dist(lastSentPos, position);
            if (distance > GPS_LOCATION_DIST_THRESHOLD)
            {
                printEvent("Moved " + String(distance) + " m.");
                SetState(STATE_GPS_MOVING);
                SendGPSMessage();
            }
            else
            {
                if (msSinceLastMessage() > GPS_LOCATION_STANDING_THRESHOLD)
                {
                    printEvent("Not moved since more than " + String(GPS_LOCATION_STANDING_THRESHOLD / 1000) + " s distance to last transmitted position is " + String(distance) + "m.");
                    SetState(STATE_DEEP_SLEEP_REQUESTED);
                }
            }
        }
        break;
    case STATE_GPS_MOVING:
        if (hadGPSFix)
        {
            pos position = {gps.location.lat(), gps.location.lng()};
            float distance = dist(lastSentPos, position);
            if (distance < GPS_LOCATION_DIST_THRESHOLD)
            {
                printEvent("Not moved more than " + String(GPS_LOCATION_DIST_THRESHOLD) + "m, distance to last transmitted position is " + String(distance) + "m.");
                SetState(STATE_GPS_STANDING);
            }
            else
            {
                SendGPSMessage();
            }
        }
        break;
    case STATE_SEND_NETWORK_MESSAGE:
        if (getMessageCount() > 0)
        {
            printEvent("Sent at least 1 message, requesting DeepSleep.");
            SetState(STATE_DEEP_SLEEP_REQUESTED);
        }
        break;

    case STATE_DEEP_SLEEP_REQUESTED:
        if (getMessageQueue() <= 0 && getMessageCount() > 0)
        {
            printEvent("Sent at least 1 message and none queued, going into DeepSleep.");
            GoDeepSleep();
        }
        break;

    default:
        printEvent("Unknown State: " + String(systemState));
        break;
    }
}

void UserDownlinkMessage(ostime_t txCompleteTimestamp, uint8_t fPort, uint8_t *data, uint8_t dataLength)
{

    if (getMessageQueue() > 0)
        return;
    // This function is called from the onEvent() event handler
    // on EV_TXCOMPLETE when a downlink message was received.

    // Implements a 'reset counter' command that can be sent via a downlink message.
    // To send the reset counter command to the node, send a downlink message
    // (e.g. from the TTN Console) with single byte value resetCmd on port cmdPort.

    const uint8_t cmdPort = 100;
    const uint8_t resetCmd = 0xC0;

    if (fPort == cmdPort && dataLength == 1 && data[0] == resetCmd)
    {
#ifdef USE_SERIAL
        printSpaces(serial, MESSAGE_INDENT);
        serial.println(F("Reset cmd received"));
#endif
        ESP.restart();
    }
}

void UserRXTXComplete()
{
    if (getMessageCount() > 0 && systemState == STATE_DEEP_SLEEP_REQUESTED)
        GoDeepSleep();
}