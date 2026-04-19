#include "net_nmea0183.h"
#include <WiFi.h>

NetNMEA0183::NetNMEA0183(const char *host, int port, tState *state)
    : host(host), port(port), state(state) {}

void NetNMEA0183::begin()
{
    connect();
}

void NetNMEA0183::run()
{
    if (WiFi.status() != WL_CONNECTED) return;

    if (!connected || !client.connected()) {
        if (connected && !client.connected()) {
            // TCP layer dropped the connection unexpectedly
            Serial.println("NMEA0183: TCP connection lost");
            state->pushConnEvent(tState::CONN_EVENT_TCP_DROP);
            disconnect();
        }
        if (millis() - lastConnectAttempt > RETRY_MS) {
            if (connect())
                state->pushConnEvent(tState::CONN_EVENT_RECONNECT);
        }
        return;
    }

    while (client.available()) {
        char c = client.read();
        if (c == '\n' || c == '\r') {
            if (lineLen > 0) {
                lineBuf[lineLen] = '\0';
                processLine(lineBuf);
                lineLen = 0;
            }
        } else if (lineLen < (int)sizeof(lineBuf) - 1) {
            lineBuf[lineLen++] = c;
        }
        lastActivity = millis();
    }

    // Timeout check — server went silent at TCP level
    if (millis() - lastActivity > TIMEOUT_MS) {
        Serial.println("NMEA0183: timeout, reconnecting");
        state->pushConnEvent(tState::CONN_EVENT_TCP_DROP);
        disconnect();
        if (connect())
            state->pushConnEvent(tState::CONN_EVENT_RECONNECT);
        return;
    }

    // Watchdog — connected but no valid NMEA sentence received
    if (millis() - lastValidData > WATCHDOG_MS) {
        Serial.println("NMEA0183: watchdog, no valid data, reconnecting");
        state->pushConnEvent(tState::CONN_EVENT_WATCHDOG);
        disconnect();
        if (connect())
            state->pushConnEvent(tState::CONN_EVENT_RECONNECT);
    }
}

bool NetNMEA0183::connect()
{
    if (strlen(host) == 0 || port <= 0) return false;
    Serial.printf("NMEA0183: connecting to %s:%d\n", host, port);
    lastConnectAttempt = millis();
    client.setTimeout(5);   // 5 s connect timeout — avoids blocking the task indefinitely
    if (client.connect(host, port)) {
        Serial.println("NMEA0183: connected");
        connected = true;
        lastActivity = millis();
        lastValidData = millis();
        lineLen = 0;
        return true;
    }
    Serial.println("NMEA0183: connection failed");
    return false;
}

void NetNMEA0183::disconnect()
{
    client.stop();
    connected = false;
}

// Returns true if NMEA checksum is valid.
// Expected format: $....*XX  (XX = hex XOR of chars between $ and *)
bool NetNMEA0183::validChecksum(const char *line)
{
    if (line[0] != '$' && line[0] != '!') return false;
    const char *star = strchr(line, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t calc = 0;
    for (const char *p = line + 1; p < star; p++) calc ^= (uint8_t)*p;
    uint8_t given = (uint8_t)strtol(star + 1, nullptr, 16);
    return calc == given;
}

// Split a comma-delimited NMEA sentence into fields.
// fields[] receives pointers into a mutable copy of line.
static int splitFields(char *buf, char *fields[], int maxFields)
{
    int n = 0;
    char *p = buf;
    while (n < maxFields) {
        fields[n++] = p;
        p = strchr(p, ',');
        if (!p) break;
        *p++ = '\0';
    }
    // Strip checksum from last field
    if (n > 0) {
        char *star = strchr(fields[n - 1], '*');
        if (star) *star = '\0';
    }
    return n;
}

// Parse degrees+minutes: "DDDMM.MMM" -> decimal degrees
static float parseDegMin(const char *s, char hemi)
{
    if (!s || s[0] == '\0') return NAN;
    float raw = atof(s);
    int deg = (int)(raw / 100);
    float min = raw - deg * 100.0f;
    float dec = deg + min / 60.0f;
    if (hemi == 'S' || hemi == 'W') dec = -dec;
    return dec;
}

void NetNMEA0183::processLine(const char *line)
{
    if (!validChecksum(line)) {
        Serial.printf("Invalid Checksum: %s\n", line);
        return;
    }

    // Any sentence with a valid checksum means the TCP link is alive.
    lastValidData = millis();

    char buf[128];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[20];
    int n = splitFields(buf, fields, 20);
    if (n < 1) return;

    // Skip '$'/'!' + 2-char talker ID, compare only the 3-char sentence type
    const char *tag = fields[0];  // e.g. "$WIMWV" or "!AIVDM"
    if ((tag[0] != '$' && tag[0] != '!') || strlen(tag) < 4) return;
    const char *type = tag + 3;   // e.g. "MWV", "RMC", "GGA" ...

    // AIS sentences (VDM/VDO) — valid but not parsed here
    if (strcmp(type, "VDM") == 0 || strcmp(type, "VDO") == 0) return;

    // xMWV,<angle>,<ref>,<speed>,<unit>,<status>
    // ref: R=relative(apparent), T=true
    if (strcmp(type, "MWV") == 0 && n >= 6 && fields[5][0] == 'A') {
        float angle = atof(fields[1]) * DEG_TO_RAD;
        char  ref   = fields[2][0];
        float speed = atof(fields[3]);
        if (fields[4][0] == 'K') speed /= 3.6f;        // km/h -> m/s
        else if (fields[4][0] == 'N') speed *= 0.514444f; // knots -> m/s
        state->onNMEA0183Wind(angle, speed, ref == 'R');
    }
    // xRMC,<time>,<status>,<lat>,<NS>,<lon>,<EW>,<sog>,<cog>,<date>,...
    else if (strcmp(type, "RMC") == 0 && n >= 10) {
        // Time/date are valid even before the GPS has a position fix (status 'V'),
        // so try to sync the clock regardless of status.
        if (fields[1][0] && fields[9][0])
            state->onNMEA0183DateTime(fields[1], fields[9]);
        // Position/speed data are only reliable with a valid fix (status 'A').
        if (fields[2][0] == 'A') {
            float lat = parseDegMin(fields[3], fields[4][0]);
            float lon = parseDegMin(fields[5], fields[6][0]);
            float sog = atof(fields[7]) * 0.514444f; // knots -> m/s
            float cog = atof(fields[8]) * DEG_TO_RAD;
            //Serial.printf("Position %0.4f %0.4f\n", lat, lon);
            state->onNMEA0183Position(lat, lon);
            state->onNMEA0183SOGCOGTrue(sog, cog);
        }
    }
    // xGGA,<time>,<lat>,<NS>,<lon>,<EW>,<fix>,...
    else if (strcmp(type, "GGA") == 0 && n >= 7 && atoi(fields[6]) > 0) {
        float lat = parseDegMin(fields[2], fields[3][0]);
        float lon = parseDegMin(fields[4], fields[5][0]);
        state->onNMEA0183Position(lat, lon);
    }
    // xVTG,<cogT>,T,<cogM>,M,<sogN>,N,<sogK>,K,...
    else if (strcmp(type, "VTG") == 0 && n >= 7) {
        float cog = atof(fields[1]) * DEG_TO_RAD;
        float sog = atof(fields[5]) * 0.514444f; // knots -> m/s
        state->onNMEA0183SOGCOGTrue(sog, cog);
    }
    // xHDG,<mag>,<dev>,<devEW>,<var>,<varEW>
    else if (strcmp(type, "HDG") == 0 && n >= 2) {
        float hdg = atof(fields[1]) * DEG_TO_RAD;
        state->onNMEA0183HeadingMagnetic(hdg);
    }
    // xHDT,<true>,T
    else if (strcmp(type, "HDT") == 0 && n >= 2) {
        float hdg = atof(fields[1]) * DEG_TO_RAD;
        state->onNMEA0183HeadingTrue(hdg);
    }
    // xXDR,<type>,<value>,<unit>,<id>[,<type>,<value>,<unit>,<id>...]
    // Angular (A) transducers: HEEL/ROLL -> roll, PTCH/PITCH -> pitch (degrees)
    else if (strcmp(type, "XDR") == 0 && n >= 5) {
        float roll  = NAN;
        float pitch = NAN;
        // Each transducer group occupies 4 fields starting at index 1
        for (int i = 1; i + 3 < n; i += 4) {
            if (fields[i][0] != 'A') continue;  // only Angular type
            const char *id  = fields[i + 3];
            float val = atof(fields[i + 1]) * DEG_TO_RAD;
            if (strcasecmp(id, "HEEL") == 0 || strcasecmp(id, "ROLL") == 0)
                roll = val;
            else if (strcasecmp(id, "PTCH") == 0 || strcasecmp(id, "PITCH") == 0)
                pitch = val;
        }
        if (!isnan(roll) || !isnan(pitch))
            state->onNMEA0183Attitude(roll, pitch);
    } else {
        //Serial.printf("Unknown sequence %s\n", line);
    }

}
