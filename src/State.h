/* State Definition */

#ifndef _State_H_
#define _State_H_

#include <SD.h>
#include "NMEA2000.h"
#include "N2kTypes.h"
#include "PyTypes.h"
#include <ArduinoJson.h>



class tState
{
protected:

    bool verbose = false;
    bool timeSet = false;

    void setupTime(time_t t);
    void setupTimeSK(String datetime);
    void handleSystemDateTime(const tN2kMsg &N2kMsg);
    void handleHeadingTrackControl(const tN2kMsg &N2kMsg);
    void handleNavigationInfo(const tN2kMsg &N2kMsg);
    void handleXTE(const tN2kMsg &N2kMsg);
    void handleRouteInfo(const tN2kMsg &N2kMsg);
    void handleWind(const tN2kMsg &N2kMsg);
    void handleRudderCommand(const tN2kMsg &N2kMsg);
    void handleHeading(const tN2kMsg &N2kMsg);
    void handleRateOfTurn(const tN2kMsg &N2kMsg);
    void handleAttitude(const tN2kMsg &N2kMsg);
    void handleMagneticVariation(const tN2kMsg &N2kMsg);
    void handlePositionRapidUpdate(const tN2kMsg &N2kMsg);
    void handleCOGSOGRapidUpdate(const tN2kMsg &N2kMsg);
    void handleGNSS(const tN2kMsg &N2kMsg);

    bool ParseN2kPGN129285(const tN2kMsg &N2kMsg, uint16_t &Start, uint16_t &nItems, uint16_t &Database, uint16_t &Route,
                                   tN2kNavigationDirection &NavDirection, char *RouteName, size_t RouteNameBufSize, tN2kGenericStatusPair &SupplementaryData,
                                   uint16_t wptArraySize, t_waypoint *waypoints);

    bool parseObj(JsonObject obj);
    void update_value(String &path, size_t &u_idx, size_t &v_idx, JsonVariant &value);
    
public:
    
    tHeadingData cog{when : 0, origin : 0, reference : tN2kHeadingReference::N2khr_Unavailable, heading : 0.0};
    tDoubleData sog{when : 0, origin : 0, value : 0.0}; // Speed in m/s
    tDoubleData stw{when : 0, origin : 0, value : 0.0}; // Speed through water in m/s
    tDoubleData depth{when : 0, origin : 0, value : 0.0}; // Depth in m
    tPositionData position{when : 0, origin : 0, latitude : 42.428205, longitude : 3.165478};
    tHeadingData heading{when : 0, origin : 0, reference : tN2kHeadingReference::N2khr_Unavailable, heading : 0.0}; // ap.heading
    tAttitudeData attitude{when : 0, origin : 0, yaw : 0.0, pitch : 0.0, roll : 0.0};
    tDoubleData rateOfTurn{when : 0, origin : 0, value : 0.0}; // degrees/s
    tDoubleData deviation{when : 0, origin : 0, value : 0.0};
    tDoubleData variation{when : 0, origin : 0, value : 0.0};
    tDoubleData rudderAngle{when : 0, origin : 0, value : 0.0};     // rudder.angle
    tWindData wind{when : 0, origin : 0, reference: tN2kWindReference::N2kWind_Apparent, speed : 0.0, angle : 0.0}; // wind.speed, wind.angle
    tDoubleData rpm{when : 0, origin : 0, value : 0.0}; // RPM
    tDoubleData engineTemperature{when : 0, origin : 0, value : 0.0}; // Engine Temperature
    tDoubleData oilPressure{when : 0, origin : 0, value : 0.0}; // Engine Temperature
    
    
    // RW , Commands and data - Not used for the moment
    tModeData mode{when : 0, origin : 0, value : tPyPilotMode::compass}; // ap.mode
    tBoolData engaged{when : 0, origin : 0, value : false};              // ap.enabled
    tDoubleData headingCommandTrue{when : 0, origin : 0, value : 0.0};
    tDoubleData headingCommandMagnetic{when : 0, origin : 0, value : 0.0}; // ap.heading_command
    tRudderCommandData rudderCommand{when : 0, origin : 0, direction : tN2kRudderDirectionOrder::N2kRDO_NoDirectionOrder, command : 0.0};

    tTackStateData tackState{when : 0, origin : 0, value : tTackState::TACK_NONE};                // ap.tack.state
    tTackDirectionData tackDirection{when : 0, origin : 0, value : tTackDirection::TACKING_NONE}; // ap.tack.direction

    // Servo Data  - Not used for the moment

    tDoubleData servoVoltage{when : 0, origin : 0, value : 0.0};        // servo.voltage
    tDoubleData servoAmpHr{when : 0, origin : 0, value : 0.0};          // servo.amp_hours
    tDoubleData servoControllerTemp{when : 0, origin : 0, value : 0.0}; // servo.controller_temp
    tDoubleData servoPosition{when : 0, origin : 0, value : 0.0};       // servo.position

    void HandleNMEA2000Msg(const tN2kMsg &N2kMsg, bool analyze, bool verbose);

    void printInfo();
    void saveCsv(File f);
    void saveGPXTrackpoint(File f);
    void saveGPXHeader(File f, char* name);
    void saveGPXFooter(File f);
    bool signalk_parse_ws(String msg);
};

#endif