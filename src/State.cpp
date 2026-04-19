#include <M5Unified.h>
#include <Arduino.h>
#include <string.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include "PyTypes.h"
#include "pgnsToString.h"
#include "State.h"
#include "Utilities.h"

extern int sources[MAX_SOURCES];
extern int n_sources;

// Given a time_t updates the RTC

void tState::setupTime(time_t t)
{

  struct tm *tm;
  m5::rtc_time_t RTCtime;
  m5::rtc_date_t RTCDate;

  tm = localtime(&t);

  RTCtime.hours = tm->tm_hour;
  RTCtime.minutes = tm->tm_min;
  RTCtime.seconds = tm->tm_sec;
  M5.Rtc.setTime(RTCtime);

  RTCDate.year = tm->tm_year + 1900;
  RTCDate.month = tm->tm_mon + 1;
  RTCDate.date = tm->tm_mday;
  M5.Rtc.setDate(RTCDate);

  // Also sync the POSIX system clock so getLocalTime() works correctly
  struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
  settimeofday(&tv, nullptr);

  timeSet = true;
}

void tState::setupTimeSK(String datetime)
{
  m5::rtc_time_t RTCtime;
  m5::rtc_date_t RTCDate;

  Serial.println("Setting date time from Signal K " + datetime);
  RTCDate.year = atoi(datetime.substring(0, 4).c_str());
  RTCDate.month = atoi(datetime.substring(5, 7).c_str());
  RTCDate.date = atoi(datetime.substring(8, 10).c_str());
  M5.Rtc.setDate(RTCDate);

  RTCtime.hours = atoi(datetime.substring(11, 13).c_str());
  RTCtime.minutes = atoi(datetime.substring(14, 16).c_str());
  RTCtime.seconds = atoi(datetime.substring(17, 19).c_str());
  M5.Rtc.setTime(RTCtime); // writes the  time to the (RTC) real time clock.

  // Also sync the POSIX system clock so getLocalTime() works correctly
  struct tm t2 = {};
  t2.tm_year = RTCDate.year - 1900;
  t2.tm_mon  = RTCDate.month - 1;
  t2.tm_mday = RTCDate.date;
  t2.tm_hour = RTCtime.hours;
  t2.tm_min  = RTCtime.minutes;
  t2.tm_sec  = RTCtime.seconds;
  time_t epoch = mktime(&t2);
  struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);

  timeSet = true;
}
/* NMEA 2000 support */

/* Route WP Information. Not used for log */

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

// Probably would be better do it once for session

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

  time_t now = SystemTime + (SystemDate * 86400);
  setupTime(now);
}

// Heading and Track Control.  We only use VesselHeading
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

    if (HeadingReference == tN2kHeadingReference::N2khr_magnetic)
    {
      magneticHeading.heading = VesselHeading;
      magneticHeading.when = time(nullptr);
      magneticHeading.origin = N2kMsg.Source;
      magneticHeading.reference = HeadingReference;
    }
    else if (HeadingReference == tN2kHeadingReference::N2khr_true)
    {
      trueHeading.heading = VesselHeading;
      trueHeading.when = time(nullptr);
      trueHeading.origin = N2kMsg.Source;
      trueHeading.reference = HeadingReference;
    }

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
void tState::handleDepth(const tN2kMsg &N2kMsg){

  unsigned char SID;
  double DepthBelowTransducer;
  double Offset;
  double Range;



  ParseN2kWaterDepth(N2kMsg, SID, DepthBelowTransducer, Offset, Range);

  depth.when = time(nullptr);
  depth.origin = N2kMsg.Source;
  depth.value = DepthBelowTransducer + Offset;

}

void tState::handleTemperature(const tN2kMsg &N2kMsg){

  unsigned char SID;
  unsigned char TempInstance;
  tN2kTempSource TempSource;
  double ActualTemperature;
  double SetTemperature;

  ParseN2kTemperature(N2kMsg, SID, TempInstance, TempSource, ActualTemperature, SetTemperature);

  
  engineTemperature.when = time(nullptr);
  engineTemperature.origin = N2kMsg.Source;
  engineTemperature.value = ActualTemperature;

}

// Rudder command is not used for the moment
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

// Navigation Info. Not used in Logbook for the moment

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

// Cross Track Error. Not used in log for the moment
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

// Route Info. See parrseN2kPGN!29285. Not used for the moment

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

// Wind

void tState::handleWind(const tN2kMsg &N2kMsg)
{
  double windSpeed;
  double windAngle;
  unsigned char SID;
  tN2kWindReference windReference;

  ParseN2kPGN130306(N2kMsg, SID, windSpeed, windAngle, windReference);

  if (windReference == tN2kWindReference::N2kWind_Apparent)
  {
    apparentWind.when = time(nullptr);
    apparentWind.origin = N2kMsg.Source;
    apparentWind.reference = windReference;
    apparentWind.angle = windAngle;
    apparentWind.speed = windSpeed;
  }
  else if (windReference == tN2kWindReference::N2kWind_True_North)
  {
    trueWind.when = time(nullptr);
    trueWind.origin = N2kMsg.Source;
    trueWind.reference = windReference;
    trueWind.angle = windAngle;
    trueWind.speed = windSpeed;
    runWindFilter((uint64_t)millis());
  }

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



// Heading. Also updates deviation and variation

void tState::handleHeading(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double Heading;
  double Deviation;
  double Variation;
  tN2kHeadingReference ref;

  ParseN2kHeading(N2kMsg, SID, Heading, Deviation, Variation, ref);

  time_t now = time(nullptr);
  
  double d = 0.0;
  if (Variation > -1000.00){
    d += Variation;
  }

  if (Deviation > -1000.00){
    d += Deviation;
  }
  
  if (ref == tN2kHeadingReference::N2khr_magnetic)
  {

    magneticHeading.when = now;
    magneticHeading.origin = N2kMsg.Source;
    magneticHeading.reference = ref;
    magneticHeading.heading = Heading;

    trueHeading.when = now;
    trueHeading.origin = N2kMsg.Source;
    trueHeading.reference = ref;
    trueHeading.heading = Heading + d;
  }
  else if (ref == tN2kHeadingReference::N2khr_true)
  {

    trueHeading.when = now;
    trueHeading.origin = N2kMsg.Source;
    trueHeading.reference = ref;
    trueHeading.heading = Heading;

     magneticHeading.when = now;
    magneticHeading.origin = N2kMsg.Source;
    magneticHeading.reference = ref;
    magneticHeading.heading = Heading - d;
  }

  variation.when = now;
  variation.origin = N2kMsg.Source;
  variation.value = Variation;

  deviation.when = now;
  deviation.origin = N2kMsg.Source;
  deviation.value = Deviation;

}

// Rate Of Turn

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

// Attitude. Usually Yaw is the same as Heading (for a constant)

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

  if (!verbose)
  {
    return;
  }
  Serial.println("Attitude");
  Serial.print("Source ");
  Serial.print(N2kMsg.Source);
  Serial.print(" Yaw ");
  Serial.print(attitude.yaw / 3.141592 * 180.0);
  Serial.print(" Pitch ");
  Serial.print(attitude.pitch / 3.141592 * 180.0);
  Serial.print(" Roll ");
  Serial.println(attitude.roll / 3.141592 * 180.0);
}

// Variation

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

// Position Rapid Update

void tState::handlePositionRapidUpdate(const tN2kMsg &N2kMsg)
{

  double Latitude;
  double Longitude;
  ParseN2kPositionRapid(N2kMsg, Latitude, Longitude);

  position.when = time(nullptr);
  position.origin = N2kMsg.Source;
  position.latitude = Latitude;
  position.longitude = Longitude;

  if (!verbose)
  {
    return;
  }
  Serial.println("Position Rapid Update");
  Serial.print("Source ");
  Serial.print(N2kMsg.Source);
  Serial.print(" Latitude");
  Serial.print(position.latitude);
  Serial.print(" Longitude ");
  Serial.println(position.longitude);
}

// COGSOG Rapid Update

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

// GNSS . Only use position for the moment. Other data may be interesting if an accident or erroneus position

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

  time_t now = SecondsSinceMidnight + (DaysSince1970 * 86400);
  setupTime(now); // Update local clock with GNSS time

  position.when = now;
  position.origin = N2kMsg.Source;
  position.latitude = Latitude;
  position.longitude = Longitude;

  // There is much much more info that I don't know if they are very useful
}

// Handle rapig engine parameters

void tState::handleEngineParamRapid(const tN2kMsg &N2kMsg)
{
  unsigned char EngineInstance;
  double EngineSpeed;
  double EngineBoostPressure;
  int8_t EngineTiltTrim;

  ParseN2kEngineParamRapid(N2kMsg,EngineInstance,EngineSpeed,EngineBoostPressure,EngineTiltTrim);

  rpm.when = time(nullptr);
  rpm.origin = N2kMsg.Source;
  rpm.value = EngineSpeed;
 
}

void tState::handleEngineDynamicParameters(const tN2kMsg &N2kMsg)
{
unsigned char EngineInstance;
double EngineOilPress;
double EngineOilTemp;
double EngineCoolantTemp;
double AltenatorVoltage;
double FuelRate;
double EngineHours;
double EngineCoolantPress;
double EngineFuelPress;
int8_t EngineLoad;
int8_t EngineTorque;

  ParseN2kEngineDynamicParam(N2kMsg, EngineInstance, EngineOilPress,
                      EngineOilTemp, EngineCoolantTemp, AltenatorVoltage,
                      FuelRate, EngineHours,EngineCoolantPress, EngineFuelPress,
                      EngineLoad, EngineTorque);

  engineTemperature.when = time(nullptr);
  engineTemperature.origin = N2kMsg.Source;
  engineTemperature.value = EngineCoolantTemp;
 
} 

/*
  This code is general and includes engine, rudder and temperature but for the wind project
  we only need :

  PGN    Source  PGN Name
129025   15    Position
129026   15    COG/SOG  
127250   15    Heading
130306   15    Wind
126992   15    Date/Time

*/
void tState::runWindFilter(uint64_t nowMs)
{
    WindKalmanResult r = windFilter_.update(trueWind.angle, trueWind.speed, nowMs);
    filteredTrueWind.when      = trueWind.when;
    filteredTrueWind.origin    = trueWind.origin;
    filteredTrueWind.reference = trueWind.reference;
    filteredTrueWind.angle     = r.angle;
    filteredTrueWind.speed     = r.speed;
}

void tState::HandleNMEA2000Msg(const tN2kMsg &N2kMsg, bool analyze, bool verbose)
{
  this->verbose = verbose;

  if (!(N2kMsg.Source == 15 ||
      (N2kMsg.Source == 100 && (N2kMsg.PGN == 127489 ||N2kMsg.PGN == 127488 ||N2kMsg.PGN == 128267 || N2kMsg.PGN == 130312 || N2kMsg.PGN == 127245)))){
        return;
      }
  switch (N2kMsg.PGN)
  {
  case 126992:
    handleSystemDateTime(N2kMsg);
    break;

    case 127237:  
    handleHeadingTrackControl(N2kMsg);
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

  case 127488:
    handleEngineParamRapid(N2kMsg);
    break;

  case 127489:
    handleEngineDynamicParameters(N2kMsg);
    break;
    
  case 128259:
    // Serial.println("Received water speed (127259)");
    break;

  case 128267:
    handleDepth(N2kMsg);
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

  case 130306:
    handleWind(N2kMsg);
    break;

  case 130312:
    handleTemperature(N2kMsg);
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
  Serial.print("Heading: Magnetic : ");
  Serial.print(magneticHeading.heading);
  Serial.print(" True : ");
  Serial.println(trueHeading.heading);
  Serial.print("Rudder Command: ");
  Serial.println(rudderCommand.command);
  Serial.print("Rudder Angle: ");
  Serial.println(rudderAngle.value);
}

/*

    Exporting data. Supports CSV and GPX formats
    May have a header and a footer.

*/

// Exports a record (an instant) to the csv file.
// All values in SI — no unit conversions.
// distance parameter is in NM (from RecordScreen), converted here to metres.

void tState::saveCsv(File f, double distance)
{
  char timebuf[32];
  struct tm timeinfo = {};
  if (!getLocalTime(&timeinfo, 0)) {
    // System clock not synced — fall back to RTC
    auto rtcDate = M5.Rtc.getDate();
    auto rtcTime = M5.Rtc.getTime();
    timeinfo.tm_year  = rtcDate.year - 1900;
    timeinfo.tm_mon   = rtcDate.month - 1;
    timeinfo.tm_mday  = rtcDate.date;
    timeinfo.tm_hour  = rtcTime.hours;
    timeinfo.tm_min   = rtcTime.minutes;
    timeinfo.tm_sec   = rtcTime.seconds;
  }
  strftime(timebuf, sizeof(timebuf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

  // time, distance(m)
  f.print(timebuf);
  f.print('\t'); f.print(distance * NM_TO_METERS, 3);

  // LON_raw, LAT_raw (degrees)
  f.print('\t'); f.print(position.longitude, 8);
  f.print('\t'); f.print(position.latitude,  8);

  // COG(rad), SOG(m/s)
  f.print('\t'); f.print(cog.heading,  6);
  f.print('\t'); f.print(sog.value,    6);

  // HDGm_raw(rad), HDG(rad), RDR_raw(rad)
  f.print('\t'); f.print(magneticHeading.heading, 6);
  f.print('\t'); f.print(trueHeading.heading,     6);
  f.print('\t'); f.print(rudderAngle.value,        6);

  // PITCH_raw(rad), HEEL_raw(rad), ROT_raw(rad/s)
  f.print('\t'); f.print(attitude.pitch, 6);
  f.print('\t'); f.print(attitude.roll,  6);
  f.print('\t'); f.print(rateOfTurn.value, 6);

  // AWA_raw(rad), AWS_raw(m/s)
  f.print('\t'); f.print(apparentWind.angle, 6);
  f.print('\t'); f.print(apparentWind.speed, 6);

  // TWD_raw(rad), TWS_raw(m/s)  — raw trueWind
  f.print('\t'); f.print(trueWind.angle, 6);
  f.print('\t'); f.print(trueWind.speed, 6);

  // DPT(m), RPM(Hz), EngTw(K)
  f.print('\t'); f.print(depth.value,              3);
  f.print('\t'); f.print(rpm.value,                3);
  f.print('\t'); f.print(engineTemperature.value,  2);

  // TWD(rad), TWS(m/s) — Kalman-filtered
  f.print('\t'); f.print(filteredTrueWind.angle, 6);
  f.print('\t'); f.print(filteredTrueWind.speed, 6);

  // Placeholder columns (not computed here)
  f.print("\t0\t0\t0\t0\t0\t0");

  f.println();
}

// Stores the csv header with field names

void tState::saveCsvHeader(File f)
{
  f.println("time\tdistance\tLON_raw\tLAT_raw\tCOG\tSOG"
            "\tHDGm_raw\tHDG\tRDR_raw"
            "\tPITCH_raw\tHEEL_raw\tROT_raw"
            "\tAWA_raw\tAWS_raw"
            "\tTWD_raw\tTWS_raw"
            "\tDPT\tRPM\tEngTw"
            "\tTWD\tTWS"
            "\tWind\tGust\tAWA\tAWS\tOM_grade\tENV_grade");
}

// Export an extended trakpt. There are a lot of particular extensions

void tState::saveGPXTrackpoint(File f, double distance)
{
  char buffer[100];
  struct tm timeinfo = {};

  if (!getLocalTime(&timeinfo, 0)) {
    auto rtcDate = M5.Rtc.getDate();
    auto rtcTime = M5.Rtc.getTime();
    timeinfo.tm_year  = rtcDate.year - 1900;
    timeinfo.tm_mon   = rtcDate.month - 1;
    timeinfo.tm_mday  = rtcDate.date;
    timeinfo.tm_hour  = rtcTime.hours;
    timeinfo.tm_min   = rtcTime.minutes;
    timeinfo.tm_sec   = rtcTime.seconds;
  }

  sprintf(buffer, "<trkpt lat=\"%f\" lon=\"%f\">", position.latitude, position.longitude);
  f.println(buffer);
  strftime(buffer, 64, "<time>%Y-%m-%dT%H:%M:%SZ</time>", &timeinfo);
  f.println(buffer);
  f.println("<extensions>");

  f.println("<pvt:ext>");
  sprintf(buffer, "<pvt:cog>%f</pvt:cog>", cog.heading / PI * 180.0);
  f.println(buffer);
  sprintf(buffer, "<pvt:sog>%f</pvt:sog>", sog.value);
  f.println(buffer);
  sprintf(buffer, "<pvt:dist>%f</pvt:dist>", distance * 1852.0);  // Distance in meters
  f.println(buffer);
  f.println("</pvt:ext>");

  f.println("<imu:ext>");
  sprintf(buffer, "<imu:hdg>%f</imu:hdg>", trueHeading.heading / PI * 180.0);
  f.println(buffer);
  sprintf(buffer, "<imu:pitch>%f</imu:pitch>", attitude.pitch / PI * 180.0);
  f.println(buffer);
  sprintf(buffer, "<imu:roll>%f</imu:roll>", attitude.roll / PI * 180.0);
  f.println(buffer);
  sprintf(buffer, "<imu:rot>%f</imu:rot>", rateOfTurn.value / PI * 180.0 * 60.0);
  f.println(buffer);
  f.println("</imu:ext>");
  f.println("<sea:ext>");
  sprintf(buffer, "<sea:awa>%f</sea:awa>", apparentWind.angle / PI * 180.0);
  f.println(buffer);
  sprintf(buffer, "<sea:aws>%f</sea:aws>", apparentWind.speed);
  f.println(buffer);
  sprintf(buffer, "<sea:rudder>%f</sea:rudder>", rudderAngle.value / PI * 180.0);
  f.println(buffer);
  sprintf(buffer, "<sea:depth>%f</sea:depth>", depth.value);
  f.println(buffer);

  f.println("</sea:ext>");
  f.println("<met:ext>");
  sprintf(buffer, "<met:gwd>%f</met:gwd>", trueWind.angle / PI * 180.0);
  f.println(buffer);
  sprintf(buffer, "<met:gws>%f</met:gws>", trueWind.speed);
  f.println(buffer);
  f.println("</met:ext>");

  f.println("<eng:ext>");
  sprintf(buffer, "<eng:tach>%f</eng:tach>", rpm.value * 60);
  f.println(buffer);
  sprintf(buffer, "<eng:temp>%f</eng:temp>", engineTemperature.value - 273.15);
  f.println(buffer);
  f.println("</eng:ext>");

  f.println("</extensions>");
  f.println("</trkpt>");
}

// Stores the GPX SMK header and the start of trk and trkseg
void tState::saveGPXHeader(File f, char *name)
{
  f.println("<?xml version=\"1.0\" encoding=\"utf-8\"?>");
  f.println("<gpx xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"");
  f.println("xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd http://www.cluetrust.com/XML/GPXDATA/1/0 http://www.cluetrust.com/Schemas/gpxdata10.xsd\"");

  f.println("xmlns:gpxdata=\"http://www.cluetrust.com/XML/GPXDATA/1/0\" version=\"1.1\"");
  f.println("xmlns:pvt=\"file:///Users/fgorina/Documents/Varis/pvt.xsd\"");
  f.println("xmlns:imu=\"file:///Users/fgorina/Documents/Varis/imu.xsd\"");
  f.println("xmlns:sea=\"file:///Users/fgorina/Documents/Varis/sea.xsd\"");
  f.println("xmlns:met=\"file:///Users/fgorina/Documents/Varis/met.xsd\"");
  f.println("xmlns:eng=\"file:///Users/fgorina/Documents/Varis/eng.xsd\"");
  f.println("xmlns=\"http://www.topografix.com/GPX/1/1\">");

  f.println("<trk>");
  f.print("<name>");
  f.print(name);
  f.println("</name>");
  f.println("<trkseg>");
}

// Stores the end of trkseg, trk and gpx

void tState::saveGPXFooter(File f)
{
  f.println("</trkseg>\n</trk>\n</gpx>");
}

// Connection event log

void tState::pushConnEvent(int code)
{
    int idx = connEventCount;   // read once (volatile)
    if (idx < MAX_CONN_EVENTS) {
        connEventQueue[idx] = code;
        connEventCount = idx + 1;   // publish after writing data
    }
}

void tState::flushConnEvents(File f, double distance, bool xmlFormat)
{
    int n = connEventCount;     // snapshot
    connEventCount = 0;         // consume all
    for (int i = 0; i < n; i++) {
        int code = connEventQueue[i];
        if (xmlFormat) {
            // Reuse saveGPXTrackpoint with the event code in <pvt:dist>
            char timebuf[32];
            struct tm timeinfo = {};
            if (!getLocalTime(&timeinfo, 0)) {
                auto d = M5.Rtc.getDate(); auto t = M5.Rtc.getTime();
                timeinfo.tm_year = d.year-1900; timeinfo.tm_mon = d.month-1;
                timeinfo.tm_mday = d.date;
                timeinfo.tm_hour = t.hours; timeinfo.tm_min = t.minutes; timeinfo.tm_sec = t.seconds;
            }
            strftime(timebuf, sizeof(timebuf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
            char buf[64];
            sprintf(buf, "<trkpt lat=\"%f\" lon=\"%f\">", position.latitude, position.longitude);
            f.println(buf);
            f.print("<time>"); f.print(timebuf); f.println("</time>");
            f.println("<extensions><pvt:ext>");
            sprintf(buf, "<pvt:dist>%d</pvt:dist>", code);
            f.println(buf);
            f.println("</pvt:ext></extensions>");
            f.println("</trkpt>");
        } else {
            // CSV: write a full row but with event code as the distance field
            saveCsv(f, (double)code);
        }
        f.flush();
    }
}

// SignalK Support

bool tState::signalk_parse_ws(String msg)
{
  JsonDocument doc;
  if (deserializeJson(doc, msg)) {
    Serial.println("SK: JSON parse error");
    return false;
  }

  JsonArray updates = doc["updates"];
  if (updates.isNull() || updates.size() == 0) return false;

  bool found = false;
  for (JsonObjectConst update : updates) {
    const char *timestamp = update["timestamp"] | "";
    if (timestamp[0] && !timeSet) {
      setupTimeSK(String(timestamp));
    }

    for (JsonObjectConst v : update["values"].as<JsonArrayConst>()) {
      const char *path = v["path"] | "";
      if (path[0]) {
        onSignalKDelta(path, v["value"]);
        found = true;
      }
    }
  }
  return found;
}

void tState::onSignalKDelta(const char *path, JsonVariantConst value)
{
  time_t now = time(nullptr);

  // --- navigation ---
  if (strcmp(path, "navigation.rateOfTurn") == 0) {
    if (value.is<float>()) {
      rateOfTurn = {now, -1, value.as<float>()};
    }
  }
  else if (strcmp(path, "navigation.headingMagnetic") == 0) {
    if (value.is<float>()) {
      magneticHeading.when = now; magneticHeading.origin = -1;
      magneticHeading.reference = tN2kHeadingReference::N2khr_magnetic;
      magneticHeading.heading = value.as<float>();
    }
  }
  else if (strcmp(path, "navigation.headingTrue") == 0) {
    if (value.is<float>()) {
      trueHeading.when = now; trueHeading.origin = -1;
      trueHeading.reference = tN2kHeadingReference::N2khr_true;
      trueHeading.heading = value.as<float>();
    }
  }
  else if (strcmp(path, "navigation.position") == 0) {
    if (value["latitude"].is<float>() && value["longitude"].is<float>()) {
      position.when = now; position.origin = -1;
      position.latitude  = value["latitude"].as<float>();
      position.longitude = value["longitude"].as<float>();
    }
  }
  else if (strcmp(path, "navigation.speedOverGround") == 0) {
    if (value.is<float>()) { sog = {now, -1, value.as<float>()}; }
  }
  else if (strcmp(path, "navigation.speedThroughWater") == 0) {
    if (value.is<float>()) { stw = {now, -1, value.as<float>()}; }
  }
  else if (strcmp(path, "navigation.courseOverGroundTrue") == 0) {
    if (value.is<float>()) {
      cog.when = now; cog.origin = -1;
      cog.reference = tN2kHeadingReference::N2khr_true;
      cog.heading = value.as<float>();
    }
  }
  else if (strcmp(path, "navigation.attitude") == 0) {
    if (value["pitch"].is<float>() && value["roll"].is<float>()) {
      attitude.when = now; attitude.origin = -1;
      attitude.pitch = value["pitch"].as<float>();
      attitude.roll  = value["roll"].as<float>();
    }
  }
  // --- environment.wind ---
  else if (strcmp(path, "environment.wind.angleApparent") == 0) {
    if (value.is<float>()) {
      apparentWind.when = now; apparentWind.origin = -1;
      apparentWind.reference = tN2kWindReference::N2kWind_Apparent;
      apparentWind.angle = value.as<float>();
    }
  }
  else if (strcmp(path, "environment.wind.angleTrueGround") == 0 ||
           strcmp(path, "environment.wind.directionTrue") == 0) {
    if (value.is<float>()) {
      trueWind.when = now; trueWind.origin = -1;
      trueWind.reference = tN2kWindReference::N2kWind_True_North;
      trueWind.angle = value.as<float>();
      runWindFilter((uint64_t)millis());
    }
  }
  else if (strcmp(path, "environment.wind.speedApparent") == 0) {
    if (value.is<float>()) {
      apparentWind.when = now; apparentWind.origin = -1;
      apparentWind.reference = tN2kWindReference::N2kWind_Apparent;
      apparentWind.speed = value.as<float>();
    }
  }
  else if (strcmp(path, "environment.wind.speedOverGround") == 0 ||
           strcmp(path, "environment.wind.speedTrue") == 0) {
    if (value.is<float>()) {
      trueWind.when = now; trueWind.origin = -1;
      trueWind.reference = tN2kWindReference::N2kWind_True_North;
      trueWind.speed = value.as<float>();
      runWindFilter((uint64_t)millis());
    }
  }
  // --- environment.depth ---
  else if (strcmp(path, "environment.depth.belowKeel") == 0 ||
           strcmp(path, "environment.depth.belowTransducer") == 0 ||
           strcmp(path, "environment.depth.belowSurface") == 0) {
    if (value.is<float>()) { depth = {now, -1, value.as<float>()}; }
  }
  // --- steering ---
  else if (strcmp(path, "steering.rudderAngle") == 0) {
    if (value.is<float>()) { rudderAngle = {now, -1, value.as<float>()}; }
  }
  // --- propulsion (engine ID is dynamic: propulsion.<id>.field) ---
  else if (starts_with(path, "propulsion.")) {
    const char *after = path + 11;          // skip "propulsion."
    const char *dot = strchr(after, '.');   // find end of engine ID
    if (dot) {
      const char *field = dot + 1;
      if (strcmp(field, "revolutions") == 0 && value.is<float>()) {
        rpm = {now, -1, value.as<float>()};
      } else if (strcmp(field, "temperature") == 0 && value.is<float>()) {
        engineTemperature = {now, -1, value.as<float>()};
      } else if (strcmp(field, "oilPressure") == 0 && value.is<float>()) {
        oilPressure = {now, -1, value.as<float>()};
      }
    }
  }
}

// NMEA 0183 over WiFi

void tState::onNMEA0183Wind(float angleRad, float speedMs, bool apparent)
{
  time_t now = time(nullptr);
  if (apparent) {
    apparentWind.when = now; apparentWind.origin = -1;
    apparentWind.reference = tN2kWindReference::N2kWind_Apparent;
    if (!isnan(angleRad)) apparentWind.angle = angleRad;
    if (!isnan(speedMs))  apparentWind.speed = speedMs;
  } else {
    trueWind.when = now; trueWind.origin = -1;
    trueWind.reference = tN2kWindReference::N2kWind_True_North;
    if (!isnan(angleRad)) trueWind.angle = angleRad;
    if (!isnan(speedMs))  trueWind.speed = speedMs;
    runWindFilter((uint64_t)millis());
  }
}

void tState::onNMEA0183Position(float lat, float lon)
{
  if (isnan(lat) || isnan(lon)) return;
  time_t now = time(nullptr);
  position.when = now; position.origin = -1;
  position.latitude  = lat;
  position.longitude = lon;
}

void tState::onNMEA0183SOGCOGTrue(float sogMs, float cogRad)
{
  time_t now = time(nullptr);
  if (!isnan(sogMs)) { sog.when = now; sog.origin = -1; sog.value = sogMs; }
  if (!isnan(cogRad)) {
    cog.when = now; cog.origin = -1;
    cog.reference = tN2kHeadingReference::N2khr_true;
    cog.heading = cogRad;
  }
}

void tState::onNMEA0183HeadingMagnetic(float hdgRad)
{
  if (isnan(hdgRad)) return;
  time_t now = time(nullptr);
  magneticHeading.when = now; magneticHeading.origin = -1;
  magneticHeading.reference = tN2kHeadingReference::N2khr_magnetic;
  magneticHeading.heading = hdgRad;
}

void tState::onNMEA0183HeadingTrue(float hdgRad)
{
  if (isnan(hdgRad)) return;
  time_t now = time(nullptr);
  trueHeading.when = now; trueHeading.origin = -1;
  trueHeading.reference = tN2kHeadingReference::N2khr_true;
  trueHeading.heading = hdgRad;
}

// hhmmss = "hhmmss.ss", ddmmyy = "ddmmyy"
void tState::onNMEA0183DateTime(const char *hhmmss, const char *ddmmyy)
{
  if (timeSet) return;
  if (!hhmmss || !ddmmyy || strlen(hhmmss) < 6 || strlen(ddmmyy) < 6) return;

  m5::rtc_time_t t;
  t.hours   = (hhmmss[0]-'0')*10 + (hhmmss[1]-'0');
  t.minutes = (hhmmss[2]-'0')*10 + (hhmmss[3]-'0');
  t.seconds = (hhmmss[4]-'0')*10 + (hhmmss[5]-'0');
  M5.Rtc.setTime(t);

  m5::rtc_date_t d;
  d.date  = (ddmmyy[0]-'0')*10 + (ddmmyy[1]-'0');
  d.month = (ddmmyy[2]-'0')*10 + (ddmmyy[3]-'0');
  d.year  = 2000 + (ddmmyy[4]-'0')*10 + (ddmmyy[5]-'0');
  M5.Rtc.setDate(d);

  // Also sync the POSIX system clock so getLocalTime() / time() work correctly
  struct tm t2 = {};
  t2.tm_year = d.year - 1900;
  t2.tm_mon  = d.month - 1;
  t2.tm_mday = d.date;
  t2.tm_hour = t.hours;
  t2.tm_min  = t.minutes;
  t2.tm_sec  = t.seconds;
  time_t epoch = mktime(&t2);
  struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);

  timeSet = true;
  Serial.println("NMEA0183: RTC and system clock set from GPS");
}

void tState::onNMEA0183Attitude(float rollRad, float pitchRad)
{
  if (isnan(rollRad) && isnan(pitchRad)) return;
  time_t now = time(nullptr);
  attitude.when   = now;
  attitude.origin = -2;  // NMEA0183 source
  if (!isnan(rollRad))  attitude.roll  = rollRad;
  if (!isnan(pitchRad)) attitude.pitch = pitchRad;
}

/* Signal K Parsing */