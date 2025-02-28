#include <M5Tough.h>
#include <string.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include "PyTypes.h"
#include "pgnsToString.h"
#include "State.h"

// Given a time_t updates the RTC

void tState::setupTime(time_t t)
{

  struct tm *tm;
  RTC_TimeTypeDef RTCtime;
  RTC_DateTypeDef RTCDate;

  tm = localtime(&t);

  RTCtime.Hours = tm->tm_hour;
  RTCtime.Minutes = tm->tm_min;
  RTCtime.Seconds = tm->tm_sec;
  M5.Rtc.SetTime(&RTCtime);

  RTCDate.Year = tm->tm_year + 1900;
  RTCDate.Month = tm->tm_mon + 1;
  RTCDate.Date = tm->tm_mday;
  M5.Rtc.SetDate(&RTCDate);
}
bool tState::ParseN2kPGN129285(const tN2kMsg &N2kMsg, uint16_t &Start, uint16_t &nItems, uint16_t &Database, uint16_t &Route,
                       tN2kNavigationDirection &NavDirection, char *RouteName, size_t RouteNameBufSize, tN2kGenericStatusPair &SupplementaryData,
                       uint16_t wptArraySize, t_waypoint *waypoints)
{

    if (N2kMsg.PGN != 129285L)
        return false;

    int index = 0;
    unsigned char c;
    Start = N2kMsg.Get2ByteUInt(index);
    nItems = N2kMsg.Get2ByteUInt(index);
    Database = N2kMsg.Get2ByteUInt(index);
    Route = N2kMsg.Get2ByteUInt(index);

    c = N2kMsg.GetByte(index);
    // Use flags to set values
    SupplementaryData = tN2kGenericStatusPair((c & 0x18) >> 3);
    NavDirection = tN2kNavigationDirection(c && 0x07);
    N2kMsg.GetVarStr(RouteNameBufSize, RouteName, index);
    c = N2kMsg.GetByte(index); // Reserved

    uint16_t wpid;
    char wpname[20] = "";
    double lat;
    double lon;
    unsigned int nameSize = 20;

    for (int i = 0; i < min(nItems, wptArraySize); i++)
    {
        nameSize = 20;
        waypoints[i].id = N2kMsg.Get2ByteUInt(index);
        N2kMsg.GetVarStr(nameSize, waypoints[i].name, index);
        waypoints[i].latitude = N2kMsg.Get4ByteDouble(1.0e-7, index);
        waypoints[i].longitude = N2kMsg.Get4ByteDouble(1.0e-7, index);
    }

    return true;
}

// PGN direct handles

/* We receive this message every second.
Will use to set the RTC
SystemDate is Days since 1 January 1970.
SystemsTime is seconds since midnight.

*/

void tState::handleSystemDateTime(const tN2kMsg &N2kMsg)
{
    unsigned char SID;
    uint16_t SystemDate;
    double SystemTime;
    tN2kTimeSource TimeSource;

    ParseN2kSystemTime(N2kMsg, SID, SystemDate,
                       SystemTime, TimeSource);

    // Compute the tiome since epoch

    time_t now = SystemTime + (SystemDate * 68400);
    setupTime(now);
}

void tState::handleHeadingTrackControl(const tN2kMsg &N2kMsg)
{
    tN2kOnOff RudderLimitExceeded;
    tN2kOnOff OffHeadingLimitExceeded;
    tN2kOnOff OffTrackLimitExceeded;
    tN2kOnOff Override;
    tN2kSteeringMode SteeringMode;
    tN2kTurnMode TurnMode;
    tN2kHeadingReference HeadingReference;
    tN2kRudderDirectionOrder CommandedRudderDirection;
    double CommandedRudderAngle;
    double HeadingToSteerCourse;
    double Track;
    double RudderLimit;
    double OffHeadingLimit;
    double RadiusOfTurnOrder;
    double RateOfTurnOrder;
    double OffTrackLimit;
    double VesselHeading;

    if (ParseN2kHeadingTrackControl(N2kMsg, RudderLimitExceeded, OffHeadingLimitExceeded, OffTrackLimitExceeded, Override, SteeringMode,
                                    TurnMode, HeadingReference, CommandedRudderDirection, CommandedRudderAngle, HeadingToSteerCourse, Track, RudderLimit,
                                    OffHeadingLimit, RadiusOfTurnOrder, RateOfTurnOrder, OffTrackLimit, VesselHeading))

    {

        // Will set Heading

        heading.when = time(nullptr);
        heading.origin = N2kMsg.Source;
        heading.heading = VesselHeading;
        heading.reference = HeadingReference;

        if (!verbose)
        {
            return;
        }
        Serial.print("Received Heading Track control Packet (127237)");
        Serial.print(" from: ");
        Serial.println(N2kMsg.Source);
        Serial.print("Rudder limit exceeded: ");
        Serial.println(onOffValues[RudderLimitExceeded]);
        Serial.print("Off heading limit exceeded: ");
        Serial.println(onOffValues[OffHeadingLimitExceeded]);
        Serial.print("Off Track limit exceeded: ");
        Serial.println(onOffValues[OffTrackLimitExceeded]);
        Serial.print("Override: ");
        Serial.println(onOffValues[Override]);

        Serial.print("Steering Mode: ");
        Serial.println(steeringModeValues[SteeringMode]);
        Serial.print("Turn Mode: ");
        Serial.println(turnModeValues[TurnMode]);
        Serial.print("Heading Reference: ");
        Serial.println(headingReferenceValues[HeadingReference]);
        Serial.print("Rudder Direction: ");
        Serial.println(rudderDirectionValues[CommandedRudderDirection]);

        Serial.print("Command Rudder Angle: ");
        Serial.println(CommandedRudderAngle);
        Serial.print("Heading to steer course: (");
        Serial.print(HeadingToSteerCourse / 3.141592 * 180);
        Serial.print(") ");
        Serial.println(HeadingToSteerCourse);
        Serial.print("Track: ");
        Serial.println(Track);

        Serial.print("Rudder limit: ");
        Serial.println(RudderLimit);

        Serial.print("Off Heading limit: ");
        Serial.println(OffHeadingLimit);

        Serial.print("Radius of Turn Order: ");
        Serial.println(RadiusOfTurnOrder);

        Serial.print("Rate of Turn Order: ");
        Serial.println(RateOfTurnOrder);

        Serial.print("Off Track limit: ");
        Serial.println(OffTrackLimit);
        Serial.print("Vessel heading: ");
        Serial.println(VesselHeading);
        Serial.println("------------------------------------------------------------------------------");
    }
}

void tState::handleNavigationInfo(const tN2kMsg &N2kMsg)
{
    unsigned char SID;
    double DistanceToWaypoint;
    tN2kHeadingReference BearingReference;
    bool PerpendicularCrossed;
    bool ArrivalCircleEntered;
    tN2kDistanceCalculationType CalculationType;
    double ETATime;
    int16_t ETADate;
    double BearingOriginToDestinationWaypoint;
    double BearingPositionToDestinationWaypoint;
    uint32_t OriginWaypointNumber;
    uint32_t DestinationWaypointNumber;
    double DestinationLatitude;
    double DestinationLongitude;
    double WaypointClosingVelocity;

    if (ParseN2kNavigationInfo(N2kMsg, SID, DistanceToWaypoint, BearingReference, PerpendicularCrossed, ArrivalCircleEntered, CalculationType,
                               ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint,
                               OriginWaypointNumber, DestinationWaypointNumber, DestinationLatitude, DestinationLongitude, WaypointClosingVelocity))
    {

        if (!verbose)
        {
            return;
        }
        Serial.print("Received Navigation Data Packet (129284)");
        Serial.print(" from: ");
        Serial.println(N2kMsg.Source);
        Serial.print("SID: ");
        Serial.println(SID);
        Serial.print("DistanceToWaypoint: ");
        Serial.println(DistanceToWaypoint);
        Serial.print("BearingReference: ");
        Serial.println(headingReferenceValues[BearingReference]);
        Serial.print("PerpendicularCrossed: ");
        Serial.println(PerpendicularCrossed);
        Serial.print("ArrivalCircleEntered: ");
        Serial.println(ArrivalCircleEntered);
        Serial.print("CalculationType: ");
        Serial.println(CalculationType);
        Serial.print("ETATime: ");
        Serial.println(ETATime);
        Serial.print("ETADate: ");
        Serial.println(ETADate);
        Serial.print("BearingOriginToDestinationWaypoint: ");
        Serial.println(BearingOriginToDestinationWaypoint / 3.14192 * 180.0, 4);
        Serial.print("BearingPositionToDestinationWaypoint: ");
        Serial.println(BearingPositionToDestinationWaypoint / 3.14192 * 180.0, 4);
        Serial.print("OriginWaypointNumber: ");
        Serial.println(OriginWaypointNumber);
        Serial.print("DestinationWaypointNumber: ");
        Serial.println(DestinationWaypointNumber);
        Serial.print("DestinationLatitude: ");
        Serial.println(DestinationLatitude, 6);
        Serial.print("DestinationLongitude: ");
        Serial.println(DestinationLongitude, 6);
        Serial.print("WaypointClosingVelocity: ");
        Serial.println(WaypointClosingVelocity);
        Serial.println("------------------------------------------------------------------------------");
    }
}
void tState::handleXTE(const tN2kMsg &N2kMsg)
{

    unsigned char SID;
    tN2kXTEMode XTEMode;
    bool NavigationTerminated;
    double XTE;

    if (ParseN2kXTE(N2kMsg, SID, XTEMode, NavigationTerminated, XTE))
    {

        if (!verbose)
        {
            return;
        }
        Serial.print("Received XTE  Packet (129283)");
        Serial.print(" from: ");
        Serial.println(N2kMsg.Source);
        Serial.print("    SID: ");
        Serial.println(SID);
        Serial.print("    XTEMode: ");
        Serial.println(xTEModeValues[XTEMode]);
        Serial.print("    NavigationTerminated: ");
        Serial.println(NavigationTerminated);
        Serial.print("    XTE: ");
        Serial.println(XTE);
        Serial.println("------------------------------------------------------------------------------");
    }
}
void tState::handleRouteInfo(const tN2kMsg &N2kMsg)
{

    uint16_t Start;
    uint16_t nItems;
    uint16_t Database;
    uint16_t Route;
    t_waypoint waypoints[10];

    tN2kNavigationDirection NavDirection;
    char RouteName[21] = "";
    tN2kGenericStatusPair SupplementaryData;

    if (ParseN2kPGN129285(N2kMsg, Start, nItems, Database, Route, NavDirection, RouteName, 20, SupplementaryData, 10, waypoints))
    {

        if (!verbose)
        {
            return;
        }
        Serial.print("Received Route Packet (129285)");
        Serial.print(" from: ");
        Serial.println(N2kMsg.Source);
        Serial.print("Start: ");
        Serial.println(Start);
        Serial.print("nItems: ");
        Serial.println(nItems);
        Serial.print("Database: ");
        Serial.println(Database);
        Serial.print("Route: ");
        Serial.println(Route);
        Serial.print("NavDirection: ");
        Serial.println(NavDirection);
        Serial.print("RouteName: ");
        Serial.println(RouteName);
        Serial.print("SupplementaryData: ");
        Serial.println(SupplementaryData);

        for (int i = 0; i < nItems; i++)
        {
            Serial.print("wp ");
            Serial.print(waypoints[i].id);
            Serial.print(" ");
            Serial.print(waypoints[i].name);
            Serial.print(" Lat ");
            Serial.print(waypoints[i].latitude, 6);
            Serial.print(" Lon ");
            Serial.println(waypoints[i].longitude, 6);
        }
    }
}

void tState::handleWind(const tN2kMsg &N2kMsg)
{
    double windSpeed;
    double windAngle;
    unsigned char SID;
    tN2kWindReference windReference;

    ParseN2kPGN130306(N2kMsg, SID, windSpeed, windAngle, windReference);

    wind.when = time(nullptr);
    wind.origin = N2kMsg.Source;
    wind.reference = windReference;
    wind.angle = windAngle;
    wind.speed = windSpeed;

    if (!verbose)
    {
        return;
    }
    Serial.print("Source ");
    Serial.print(N2kMsg.Source);
    Serial.print(" Wind Angle ");
    Serial.print(windAngle / 3.141592 * 180.0);
    Serial.print(" Wind Speed ");
    Serial.print(windSpeed / 1852.0 * 3600.0);
    Serial.print(" reference ");
    Serial.println(windReference);
}
void tState::handleRudderCommand(const tN2kMsg &N2kMsg)
{

    double RudderPosition;
    unsigned char Instance;
    tN2kRudderDirectionOrder RudderDirectionOrder;
    double AngleOrder;
    ParseN2kRudder(N2kMsg, RudderPosition, Instance, RudderDirectionOrder, AngleOrder);

    rudderAngle.when = time(nullptr);
    rudderAngle.origin = N2kMsg.Source;
    rudderAngle.value = RudderPosition;

    // Disregard order!!!
}

void tState::handleHeading(const tN2kMsg &N2kMsg)
{
    unsigned char SID;
    double Heading;
    double Deviation;
    double Variation;
    tN2kHeadingReference ref;

    ParseN2kHeading(N2kMsg, SID, Heading, Deviation, Variation, ref);

    time_t now = time(nullptr);

    heading.when = now;
    heading.origin = N2kMsg.Source;
    heading.reference = ref;
    heading.heading = Heading;

    variation.when = now;
    variation.origin = N2kMsg.Source;
    variation.value = Variation;

    deviation.when = now;
    deviation.origin = N2kMsg.Source;
    deviation.value = Deviation;
}

void tState::handleRateOfTurn(const tN2kMsg &N2kMsg)
{

    unsigned char SID;
    tN2kMagneticVariation Source;

    double RateOfTurn;

    ParseN2kRateOfTurn(N2kMsg, SID, RateOfTurn);

    rateOfTurn.when = time(nullptr);
    rateOfTurn.origin = N2kMsg.Source;
    rateOfTurn.value = RateOfTurn;
}
void tState::handleAttitude(const tN2kMsg &N2kMsg)
{
    unsigned char SID;
    double Yaw;
    double Pitch;
    double Roll;

    ParseN2kAttitude(N2kMsg, SID, Yaw, Pitch, Roll);

    attitude.when = time(nullptr);
    attitude.origin = N2kMsg.Source;
    attitude.yaw = Yaw;
    attitude.pitch = Pitch;
    attitude.roll = Roll;
}
void tState::handleMagneticVariation(const tN2kMsg &N2kMsg)
{

    unsigned char SID;
    tN2kMagneticVariation Source;
    uint16_t DaysSince1970;
    double Variation;

    ParseN2kMagneticVariation(N2kMsg, SID, Source, DaysSince1970, Variation);

    variation.when = time(nullptr);
    variation.origin = N2kMsg.Source;
    variation.value = Variation;
}

void tState::handlePositionRapidUpdate(const tN2kMsg &N2kMsg)
{

    double Latitude;
    double Longitude;
    ParseN2kPositionRapid(N2kMsg, Latitude, Longitude);

    position.when = time(nullptr);
    position.origin = N2kMsg.Source;
    position.latitude = Latitude;
    position.longitude = Longitude;
}

void tState::handleCOGSOGRapidUpdate(const tN2kMsg &N2kMsg)
{

    unsigned char SID;
    tN2kHeadingReference ref;
    double COG;
    double SOG;
    ParseN2kCOGSOGRapid(N2kMsg, SID, ref, COG, SOG);

    cog.when = time(nullptr);
    cog.origin = N2kMsg.Source;
    cog.reference = ref;
    cog.heading = COG;

    sog.when = time(nullptr);
    sog.origin = N2kMsg.Source;
    sog.value = SOG;
}

void tState::handleGNSS(const tN2kMsg &N2kMsg)
{
    unsigned char SID;
    uint16_t DaysSince1970;
    double SecondsSinceMidnight;

    double Latitude;
    double Longitude;
    double Altitude;

    tN2kGNSStype GNSStype;
    tN2kGNSSmethod GNSSmethod;

    unsigned char nSatellites;
    double HDOP;
    double PDOP;
    double GeoidalSeparation;

    unsigned char nReferenceStations;
    tN2kGNSStype ReferenceStationType;
    uint16_t ReferenceSationID;
    double AgeOfCorrection;

    ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight,
                 Latitude, Longitude, Altitude,
                 GNSStype, GNSSmethod,
                 nSatellites, HDOP, PDOP, GeoidalSeparation,
                 nReferenceStations, ReferenceStationType, ReferenceSationID,
                 AgeOfCorrection);

    time_t now = SecondsSinceMidnight + (DaysSince1970 * 68400);
    setupTime(now); // Update local clock with GNSS time

    position.when = now;
    position.origin = N2kMsg.Source;
    position.latitude = Latitude;
    position.longitude = Longitude;

    // There is much much more info that I don't know if they are very useful
}

void tState::HandleNMEA2000Msg(const tN2kMsg &N2kMsg, bool analyze, bool verbose)
{
    this->verbose = verbose;

    switch (N2kMsg.PGN)
    {
    case 126992:
        handleSystemDateTime(N2kMsg);

        break;
    case 127245:
        // Serial.println("Received rudder angle info (127245  )");
        handleRudderCommand(N2kMsg);
        break;

    case 127250:
        handleHeading(N2kMsg);
        break;

    case 127251:
        handleRateOfTurn(N2kMsg);
        break;

    case 127257:
        handleAttitude(N2kMsg);
        break;

    case 127258:
        handleMagneticVariation(N2kMsg);
        break;

    case 128259:
        // Serial.println("Received water speed (127259)");
        break;

    case 129025:
        handlePositionRapidUpdate(N2kMsg);
        break;

    case 129026:
        handleCOGSOGRapidUpdate(N2kMsg);
        break;

    case 129029:
        handleGNSS(N2kMsg);
        break;

    case 129283:
        handleXTE(N2kMsg);
        break;

    case 129284:
        handleNavigationInfo(N2kMsg);
        break;

    case 129285:
        handleRouteInfo(N2kMsg);
        break;

    case 127237:
        handleHeadingTrackControl(N2kMsg);
        break;

    case 130306:
        handleWind(N2kMsg);
        break;

    default:
        if (analyze)
        {
            Serial.print("Received PGN ");
            Serial.print(N2kMsg.PGN);
            Serial.print(" ");
            Serial.println(toStringPgn(N2kMsg.PGN));
        }
    }
}

void tState::tState::printInfo()
{
    Serial.println("--------------- State ------------------");
    Serial.print("Engaged: ");
    Serial.println(engaged.value);
    Serial.print("Mode: ");
    Serial.println(mode.value);
    Serial.print("Heading Command True: ");
    Serial.println(headingCommandTrue.value);
    Serial.print("Heading Command Magnetic: ");
    Serial.println(headingCommandMagnetic.value);
    Serial.print("Heading: ");
    Serial.println(heading.heading);
    Serial.print("Rudder Command: ");
    Serial.println(rudderCommand.command);
    Serial.print("Rudder Angle: ");
    Serial.println(rudderAngle.value);
}
