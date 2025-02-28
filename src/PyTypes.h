#ifndef _PyTypes_H_
#define _PyTypes_H_


typedef enum  {
    compass = 0,
    gps = 1,
    wind = 2,
    trueWind = 3,
    nav = 4 // Not really a PyPilot but simulated here
} tPyPilotMode;

typedef enum  {
    Standby = 0,
    Compass = 0x40,
    Wind = 0x100,
    Track = 0x180,
    NoDrift = 0x181
} tRaymarineMode;

typedef enum {
    TACKING_TO_PORT = -1,
    TACKING_NONE = 0,
    TACKING_TO_STARBOARD = 1
  } tTackDirection;

typedef enum  {
    TACK_NONE = 0,
    TACK_BEGIN = 1,
    TACK_WAITING = 2,
    TACK_TACKING = 3
  } tTackState;

  typedef int tDataOrigin;

  typedef struct {
    time_t when;
    tDataOrigin origin;
    double value;
  } tDoubleData;

  typedef struct {
    time_t when;
    tDataOrigin origin;
    bool value;
  } tBoolData;

  typedef struct {
    time_t when;
    tDataOrigin origin;
    tPyPilotMode value;
  } tModeData;

typedef struct {
    time_t when;
    tDataOrigin origin;
    tN2kHeadingReference reference;
    double heading;
  } tHeadingData;
  
typedef struct {
    time_t when;
    tDataOrigin origin;
    tN2kRudderDirectionOrder direction;
    double command;
  } tRudderCommandData;

 typedef struct {
    time_t when;
    tDataOrigin origin;
    tTackState value;
  } tTackStateData;

typedef struct {
    time_t when;
    tDataOrigin origin;
    tTackDirection value;
  } tTackDirectionData;

  typedef struct {
    time_t when;
    tDataOrigin origin;
    tN2kWindReference reference;
    double speed;
    double angle;
  } tWindData;  // windAngle is in rad, wind speed is in m/s

typedef struct {
    time_t when;
    tDataOrigin origin;
    double latitude;
    double longitude;
  } tPositionData;

  typedef struct {
    time_t when;
    tDataOrigin origin;
    double yaw;
    double pitch;
    double roll;
  } tAttitudeData;


  

typedef struct t_waypoint
{
  uint16_t id;
  char name[20];
  double latitude;
  double longitude;
} t_waypoint;



#endif