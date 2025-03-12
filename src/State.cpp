#include <M5Tough.h>
#include <Arduino.h>
#include <string.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include "PyTypes.h"
#include "pgnsToString.h"
#include "State.h"
#include "Utilities.h"

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
  timeSet = true;
}

void tState::setupTimeSK(String datetime)
{
  RTC_TimeTypeDef RTCtime;
  RTC_DateTypeDef RTCDate;

  Serial.println("Setting date time from Signal K " + datetime);
  RTCDate.Year = atoi(datetime.substring(0, 4).c_str());
  RTCDate.Month = atoi(datetime.substring(5, 7).c_str());
  RTCDate.Date = atoi(datetime.substring(8, 10).c_str());
  M5.Rtc.SetDate(&RTCDate);

  RTCtime.Hours = atoi(datetime.substring(11, 13).c_str());
  RTCtime.Minutes = atoi(datetime.substring(14, 16).c_str());
  RTCtime.Seconds = atoi(datetime.substring(17, 19).c_str());
  M5.Rtc.SetTime(&RTCtime); // writes the  time to the (RTC) real time clock.
  timeSet = true;
}
/* NMEA 2000 support */

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

    if (HeadingReference == tN2kHeadingReference::N2khr_magnetic)
    {
      magneticHeading.heading = VesselHeading;
      magneticHeading.when = time(nullptr);
      magneticHeading.origin = N2kMsg.Source;
      magneticHeading.reference = HeadingReference;

    }
    else  if (HeadingReference == tN2kHeadingReference::N2khr_true)
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

  if(windReference == tN2kWindReference::N2kWind_Apparent){
    apparentWind.when = time(nullptr);
    apparentWind.origin = N2kMsg.Source;
    apparentWind.reference = windReference;
    apparentWind.angle = windAngle;
    apparentWind.speed = windSpeed;
  }else if (windReference == tN2kWindReference::N2kWind_True_North){
    trueWind.when = time(nullptr);
    trueWind.origin = N2kMsg.Source;
    trueWind.reference = windReference;
    trueWind.angle = windAngle;
    trueWind.speed = windSpeed;
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

  if(ref == tN2kHeadingReference::N2khr_magnetic){
    magneticHeading.when = now;
    magneticHeading.origin = N2kMsg.Source;
    magneticHeading.reference = ref;
    magneticHeading.heading = Heading;
  }else if(ref == tN2kHeadingReference::N2khr_true){
    trueHeading.when = now;
    trueHeading.origin = N2kMsg.Source;
    trueHeading.reference = ref;
    trueHeading.heading = Heading;
  }


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

*/

void tState::saveCsv(File f)
{
  char buffer[100];
  struct tm timeinfo;
  getLocalTime(&timeinfo);

  strftime(buffer, 64, "%Y-%m-%dT%H:%M:%SZ", (const tm *)&timeinfo);
  sprintf(buffer, "%s\t", buffer);
  f.print(buffer);
  sprintf(buffer, "%f\t%f\t", position.longitude, position.latitude);
  f.print(buffer);
  sprintf(buffer, "%f\t%f\t", cog.heading / PI * 180.0, sog.value * 3600.0 / 1852.0);
  f.print(buffer);
  sprintf(buffer, "%f\t%f\t%f\t%f\t%f\t", magneticHeading.heading / PI * 180.0, trueHeading.heading / PI * 180.0, attitude.pitch / PI * 180.0, attitude.roll / PI * 180.0, rateOfTurn.value / PI * 180.0);
  f.print(buffer);
  sprintf(buffer, "%f\t%f\t", apparentWind.angle / PI * 180.0, apparentWind.speed * 3600.0 / 1852.0);
  f.print(buffer);
  sprintf(buffer, "%f\t%f\t", trueWind.angle / PI * 180.0, trueWind.speed * 3600.0 / 1852.0);
  f.print
  (buffer);
  sprintf(buffer, "%f", depth.value);
  f.println(buffer);
}

void tState::saveCsvHeader(File f)
{
  f.println("UTC\tLongitude\tLatitude\tCOG\tSOG\tHeading Mag\tHeading True\tPitch\tRoll\tRateOfTurn\tAWA\tAWS\tTWD\tTWS\tDepth");
}
void tState::saveGPXTrackpoint(File f)
{
  char buffer[100];
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo))
  {
    Serial.println("No tinc dades de temps");
  }

  sprintf(buffer, "<trkpt lat=\"%f\" lon=\"%f\">", position.latitude, position.longitude);
  f.println(buffer);
  strftime(buffer, 64, "<time>%Y-%m-%dT%H:%M:%SZ</time>", (const tm *)&timeinfo);
  f.println(buffer);
  f.println("<extensions>");

  f.println("<pvt:ext>");
  sprintf(buffer, "<pvt:cog>%f</pvt:cog>", cog.heading);
  f.println(buffer);
  sprintf(buffer, "<pvt:sog>%f</pvt:sog>", sog.value);
  f.println(buffer);
  f.println("</pvt:ext>");

  f.println("<imu:ext>");
  sprintf(buffer, "<imu:hdg>%f</imu:hdg>", trueHeading.heading);
  f.println(buffer);
  sprintf(buffer, "<imu:pitch>%f</imu:pitch>", attitude.pitch);
  f.println(buffer);
  sprintf(buffer, "<imu:roll>%f</imu:roll>", attitude.roll);
  f.println(buffer);
  sprintf(buffer, "<imu:rot>%f</imu:rot>", rateOfTurn.value);
  f.println(buffer);
  f.println("</imu:ext>");
  f.println("<sea:ext>");
  sprintf(buffer, "<sea:awa>%f</sea:awa>", apparentWind.angle);
  f.println(buffer);
  sprintf(buffer, "<sea:aws>%f</sea:aws>", apparentWind.speed);
  f.println(buffer);
  
  sprintf(buffer, "<sea:depth>%f</sea:depth>", depth.value);
  f.println(buffer);

  f.println("</sea:ext>");
  f.println("<met:ext>");
  sprintf(buffer, "<met:gwd>%f</met:gwd>", trueWind.angle);
  f.println(buffer);
  sprintf(buffer, "<met:gws>%f</met:gws>", trueWind.speed);
  f.println(buffer);
  f.println("</met:ext>");


  f.println("</extensions>");
  f.println("</trkpt>");
}

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
  f.println("xmlns=\"http://www.topografix.com/GPX/1/1\">");

  f.println("<trk>");
  f.print("<name>");
  f.print(name);
  f.println("</name>");
  f.println("<trkseg>");
}

void tState::saveGPXFooter(File f)
{
  f.println("</trkseg>\n</trk>\n</gpx>");
}


// SignalK Support

bool tState::signalk_parse_ws(String msg)
{
  bool found = false;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, msg);
  // Parse succeeded?

  if (err)
  {
    Serial.println("Error decoding message");
    Serial.println(msg);
    return false;
  }

  JsonObject obj = doc.as<JsonObject>();

  if (obj != NULL)
  {
    found = parseObj(obj);
  }
  return found;
}

bool tState::parseObj(JsonObject obj)
{
  bool found = false;
  JsonArray updates = obj["updates"];
  if (updates != NULL)
  {
    for (size_t i_u = 0; i_u < updates.size(); i_u++)
    {
      JsonObject update = updates[i_u];
      if (update != NULL)
      {
        JsonArray values = update["values"];
        if (values != NULL)
        {
          for (size_t i_v = 0; i_v < values.size(); i_v++)
          {
            JsonObject valueObj = values[i_v];
            if (valueObj != NULL)
            {
              if (valueObj["path"].is<const char *>())
              {
                String path = valueObj["path"].as<const char *>();
                if (path != NULL)
                {

                  JsonVariant value = valueObj["value"];
                  // Serial.print("Value "); Serial.println(value.as<int>());
                  //  PACO Removed the test of NULL becasuse when value == 0 (an integer) fails!!!
                  // if (value != NULL) {
                  update_value(path, i_u, i_v, value);
                  found = true;
                  // }
                }
              }
            }
          }
        }
      }
    }
  }
  return found;
}

void tState::update_value(String &path, size_t &u_idx, size_t &v_idx, JsonVariant &value)
{
  time_t now = time(nullptr);

  const char *p = path.c_str();
  if (starts_with(p, "navigation."))
  {

    const char *t = step_into_token(p);
    if (strcmp(t, "rateOfTurn") == 0)
    {
      if (value.is<float>())
      {
        rateOfTurn.origin = -1;
        rateOfTurn.when = now;
        rateOfTurn.value = value.as<float>();
      }
    }
    else if (strcmp(t, "headingMagnetic") == 0)
    {
      if (value.is<float>())
      {
        magneticHeading.origin = -1;
        magneticHeading.when = now;
        magneticHeading.reference = tN2kHeadingReference::N2khr_magnetic;
        magneticHeading.heading = value.as<float>();
      }
    }
    else if (strcmp(t, "headingTrue") == 0)
    {
      if (value.is<float>())
      {
        trueHeading.origin = -1;
        trueHeading.when = now;
        trueHeading.reference = tN2kHeadingReference::N2khr_true;
        trueHeading.heading = value.as<float>();
      }
    }
    else if (strcmp(t, "position") == 0)
    {

      if (value["longitude"].is<float>() && value["latitude"].is<float>())
      {
        position.when = now;
        position.origin = -1;
        position.latitude = value["latitude"].as<float>();
        position.longitude = value["longitude"].as<float>();
      }
    }
    else if (strcmp(t, "speedOverGround") == 0)
    {
      if (value.is<float>())
      {
        sog.when = now;
        sog.origin = -1;
        sog.value = value.as<float>();
      }
    }
    else if (strcmp(t, "speedThroughWater") == 0)
    {
      if (value.is<float>())
      {
        stw.when = now;
        stw.origin = -1;
        stw.value = value.as<float>();
      }
    }
    else if (strcmp(t, "courseOverGroundTrue") == 0)
    {
      if (value.is<float>())
      {
        cog.when = now;
        cog.origin = -1;
        cog.heading = value.as<float>();
        cog.reference = tN2kHeadingReference::N2khr_true;
      }
    }

    /*
    else if (strcmp(t, "courseRhumbline.crossTrackError") == 0)
    {
      if (value.is<float>())
      {
        shipDataModel.navigation.course_rhumbline.cross_track_error.m = value.as<float>();
        shipDataModel.navigation.course_rhumbline.cross_track_error.age = millis();
      }
    }
    else if (strcmp(t, "courseRhumbline.bearingTrackTrue") == 0)
    {
      if (value.is<float>())
      {
        shipDataModel.navigation.course_rhumbline.bearing_track_true.deg = value.as<float>() * 180.0 / PI;
        shipDataModel.navigation.course_rhumbline.bearing_track_true.age = millis();
      }
    }
    else if (strcmp(t, "courseRhumbline.nextPoint.distance") == 0)
    {
      if (value.is<float>())
      {
        shipDataModel.navigation.course_rhumbline.next_point.distance.m = value.as<float>();
        shipDataModel.navigation.course_rhumbline.next_point.distance.age = millis();
      }
    }
    else if (strcmp(t, "courseRhumbline.nextPoint.velocityMadeGood") == 0)
    {
      if (value.is<float>())
      {
        shipDataModel.navigation.course_rhumbline.next_point.velocity_made_good.kn = value.as<float>() / _GPS_MPS_PER_KNOT;
        shipDataModel.navigation.course_rhumbline.next_point.velocity_made_good.age = millis();
      }
    }
    else if (strcmp(t, "state") == 0)
    {
      if (value.is<String>())
      {
        String val = value.as<String>();
        if (val != NULL)
        {
          set_vessel_nav_state(val);
        }
      }
    }
      */
    else if (strcmp(t, "attitude") == 0)
    {

      // PACO Add pitch and roll from signalk

      if (value["pitch"].is<float>() && value["roll"].is<float>())
      {
        attitude.when = now;
        attitude.origin = -1;
        attitude.pitch = value["pitch"].as<float>();
        attitude.roll = value["roll"].as<float>();
      }
    }
    // Change so rtc time is initialized to UTC from GPS. Should be quite exact
    else if (strcmp(t, "datetime") == 0)
    {
      if (value.is<String>())
      {
        String val = value.as<String>();
        if (val != NULL && !timeSet)
        {
          setupTimeSK(val);
         
        }
      }
    }
  }
  else if (starts_with(p, "environment."))
  {

    const char *t = step_into_token(p);
    if (starts_with(t, "wind."))
    {
      const char *w = step_into_token(t);
      if (strcmp(w, "angleApparent") == 0)
      {

        if (value.is<float>())
        {
          apparentWind.when = now;
          apparentWind.origin = -1;
          apparentWind.angle = value.as<float>();
          apparentWind.reference = tN2kWindReference::N2kWind_Apparent;
          
        }
      }
      else if (strcmp(w, "angleTrueGround") == 0)
      {
        if (value.is<float>())
        {
          trueWind.when = now;
          trueWind.origin = -1;
          trueWind.angle = value.as<float>();
          trueWind.reference = tN2kWindReference::N2kWind_True_North;
        }
      }
      /* else if (strcmp(w, "angleTrueWater") == 0)
      {
        if (value.is<float>())
        {
          shipDataModel.environment.wind.true_wind_angle.deg = value.as<float>() * 180.0 / PI;
          shipDataModel.environment.wind.true_wind_angle.age = millis();
        }
      } */
      else if (strcmp(w, "speedApparent") == 0)
      {
        if (value.is<float>())
        {
          apparentWind.when = now;
          apparentWind.origin = -1;
          apparentWind.speed = value.as<float>();
          apparentWind.reference = tN2kWindReference::N2kWind_Apparent;
          
        }
      }
      else if (strcmp(w, "speedOverGround") == 0)
      {
        if (value.is<float>())
        {
          trueWind.when = now;
          trueWind.origin = -1;
          trueWind.speed = value.as<float>();
          trueWind.reference = tN2kWindReference::N2kWind_True_North;
        }
      }
      else if (strcmp(w, "speedTrue") == 0)
      {
        if (value.is<float>())
        {
          trueWind.when = now;
          trueWind.origin = -1;
          trueWind.speed = value.as<float>();
          trueWind.reference = tN2kWindReference::N2kWind_True_North;
        }
      }
    }
    else if (starts_with(t, "depth."))
    {
      const char *d = step_into_token(t);
      if (strcmp(d, "belowKeel") == 0)
      {
        if (value.is<float>())
        {
          depth.when = now;
          depth.origin = -1;
          depth.value = value.as<float>();
        }
      }
      else if (strcmp(d, "belowTransducer") == 0)
      {
        if (value.is<float>())
        {
          depth.when = now;
          depth.origin = -1;
          depth.value = value.as<float>();
        }
      }
      else if (strcmp(d, "belowSurface") == 0)
      {
        if (value.is<float>())
        {
          depth.when = now;
          depth.origin = -1;
          depth.value = value.as<float>();
        }
      }
    }

    else if (starts_with(p, "steering."))
    {
      const char *t = step_into_token(p);
      if (strcmp(t, "rudderAngle") == 0)
      {
        if (value.is<float>())
        {
          rudderAngle.when = now;
          rudderAngle.origin = -1;
          rudderAngle.value = value.as<float>();
        }
      }
    }
    else if (starts_with(p, "propulsion."))
    {
      String engineID = path.substring(11);
      int idx = engineID.indexOf('.');
      if (idx > 0)
      {
        engineID = engineID.substring(0, idx);
        if (engineID != NULL)
        {
          // engine_t *eng = lookup_engine(engineID.c_str());
          // if (eng != NULL)
          //{
          String prefix = String("propulsion.") + engineID;
          if (path == (prefix + ".revolutions"))
          {
            if (value.is<float>())
            {
              rpm.when = now;
              rpm.origin = -1;
              rpm.value = value.as<float>();
            }
          }
          else if (path == (prefix + ".temperature"))
          {
            if (value.is<float>())
            {
              engineTemperature.when = now;
              engineTemperature.origin = -1;
              engineTemperature.value = value.as<float>();
            }
          }
          else if (path == (prefix + ".oilPressure"))
          {
            if (value.is<float>())
            {
              oilPressure.when = now;
              oilPressure.origin = -1;
              oilPressure.value = value.as<float>();
            }
          }
        }
      }
    }
  }
  };

  /*
      else if (strcmp(t, "lights.navigation.state") == 0)
      {

        if (value.as<int>() == 0)
        {
          shipDataModel.navigation.lights.bow_red_green.state.st = on_off_e::OFF;
        }
        else
        {
          shipDataModel.navigation.lights.bow_red_green.state.st = ON;
        }
        shipDataModel.navigation.lights.bow_red_green.state.age = millis();
      }
      else if (strcmp(t, "lights.anchor.state") == 0)
      {
        if (value.as<int>() == 0)
        {
          shipDataModel.navigation.lights.anchor.state.st = on_off_e::OFF;
        }
        else
        {
          shipDataModel.navigation.lights.anchor.state.st = ON;
        }
        shipDataModel.navigation.lights.anchor.state.age = millis();
      }
      else if (strcmp(t, "lights.motoring.state") == 0)
      {
        if (value.as<int>() == 0)
        {
          shipDataModel.navigation.lights.motoring.state.st = on_off_e::OFF;
        }
        else
        {
          shipDataModel.navigation.lights.motoring.state.st = ON;
        }
        shipDataModel.navigation.lights.motoring.state.age = millis();
      }
      else if (strcmp(t, "lights.deck.state") == 0)
      {
        if (value.as<int>() == 0)
        {
          shipDataModel.navigation.lights.deck.state.st = on_off_e::OFF;
        }
        else
        {
          shipDataModel.navigation.lights.deck.state.st = ON;
        }
        shipDataModel.navigation.lights.deck.state.age = millis();
      }
      else if (strcmp(t, "lights.instruments.state") == 0)
      {
        if (value.as<int>() == 0)
        {
          shipDataModel.navigation.lights.instruments.state.st = on_off_e::OFF;
        }
        else
        {
          shipDataModel.navigation.lights.instruments.state.st = ON;
        }
        shipDataModel.navigation.lights.instruments.state.age = millis();
      }
        }
        */

  /*
else if (starts_with(p, "tanks."))
{
  String tankType;
  int idTankType;
  int idx;
  String attr;

  char *pch;
  pch = strtok((char *)p, ".");
  if (pch != NULL)
  {
    pch = strtok(NULL, ".");
    if (pch != NULL)
    {
      tankType = String(pch);
      pch = strtok(NULL, ".");
      if (pch != NULL)
      {
        idx = atoi(pch) - 1;
        pch = strtok(NULL, ".");
        if (pch != NULL)
        {
          attr = String(pch);
        }
      }
    }
  }

  if (tankType == "freshWater")
  {
    idTankType = FRESH_WATER;
  }
  else if (tankType == "fuel")
  {
    idTankType = FUEL;
  }
  else if (tankType == "wasteWater")
  {
    idTankType = WASTE_WATER;
  }
  else if (tankType == "blackWater")
  {
    idTankType = BLACK_WATER;
  }
  else
  {
    idTankType = FLUID_TYPE_NA;
  }

  if (idx < MAX_TANKS && idx >= 0)
  {
    if (attr == "currentLevel")
    {
      shipDataModel.tanks.tank[idx].percent_of_full.pct = value.as<float>() * 100.0;
      shipDataModel.tanks.tank[idx].percent_of_full.age = millis();
      shipDataModel.tanks.tank[idx].fluid_type = (fluid_type_e)idTankType;
    }
    else if (attr == "capacity")
    {
      shipDataModel.tanks.tank[idx].volume.L = value.as<float>() * 1000.0;
      shipDataModel.tanks.tank[idx].volume.age = millis();
      shipDataModel.tanks.tank[idx].fluid_type = (fluid_type_e)idTankType;
    }
  }
  Serial.print("Tank ");
  Serial.print(tankType);
  Serial.print(" Index ");
  Serial.print(idx);
  Serial.print(" Attr ");
  Serial.print(attr);
  Serial.print(" Value ");
  Serial.println(value.as<String>());
  }
  /*else if (starts_with(p, "electrical.lights."))
{

  if (strcmp(p, "electrical.lights.1.compass.state") == 0)
  {
    float f = value.as<float>();
    shipDataModel.navigation.lights.compass.intensity = f;
    shipDataModel.navigation.lights.compass.age = millis();
    Serial.print("Receiving compass light data ");
    Serial.println(f);
  }
}

  /*
        else if (path == (prefix + ".alternatorVoltage"))
        {
          if (value.is<float>())
          {
            eng->alternator_voltage.volt = value.as<float>();
            eng->alternator_voltage.age = millis();
          }
        }

         /*
  else if (starts_with(t, "outside."))
  {
    const char *o = step_into_token(t);
    if (strcmp(o, "pressure") == 0)
    {
      if (value.is<float>())
      {
        shipDataModel.environment.air_outside.pressure.hPa = value.as<float>() / 100.0;
        shipDataModel.environment.air_outside.pressure.age = millis();
      }
    }
    else if (strcmp(o, "humidity") == 0)
    {
      if (value.is<float>())
      {
        shipDataModel.environment.air_outside.humidity_pct.pct = value.as<float>() * 100.0;
        shipDataModel.environment.air_outside.humidity_pct.age = millis();
      }
    }
    else if (strcmp(o, "temperature") == 0)
    {
      if (value.is<float>())
      {
        shipDataModel.environment.air_outside.temp_deg_C.deg_C = value.as<float>() - 273.15;
        shipDataModel.environment.air_outside.temp_deg_C.age = millis();
      }
    }
    else if (strcmp(o, "illuminance") == 0)
    {
      if (value.is<float>())
      {
        shipDataModel.environment.air_outside.illuminance.lux = value.as<float>();
        shipDataModel.environment.air_outside.illuminance.age = millis();
      }
    }
  }
}

  */

  /* Signal K Parsing */