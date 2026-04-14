#ifndef NET_NMEA0183_H
#define NET_NMEA0183_H

#include <Arduino.h>
#include <WiFiClient.h>
#include "State.h"

class NetNMEA0183
{
public:
    NetNMEA0183(const char *host, int port, tState *state);
    void begin();
    void run();

private:
    const char *host;
    int port;
    tState *state;

    WiFiClient client;
    bool connected = false;
    unsigned long lastActivity = 0;
    unsigned long lastValidData = 0;
    unsigned long lastConnectAttempt = 0;
    static const unsigned long TIMEOUT_MS = 10000;
    static const unsigned long WATCHDOG_MS = 5000;
    static const unsigned long RETRY_MS = 2000;

    char lineBuf[128];
    int  lineLen = 0;

    bool connect();
    void disconnect();
    void processLine(const char *line);
    bool validChecksum(const char *line);
};

#endif
