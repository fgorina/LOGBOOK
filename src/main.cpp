#define TFT_HOR_RES 320
#define TFT_VER_RES 240

#define ESP32_CAN_TX_PIN GPIO_NUM_19
#define ESP32_CAN_RX_PIN GPIO_NUM_27

#include <Arduino.h>
#include <M5Tough.h>
#include <CUF_24.h>
#include <time.h>
#include <NMEA2000_esp32.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include "N2kDeviceList.h"

#include "PyTypes.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include "State.h"

#include <SD.h>

// Screens

#include "MenuScreen.h"
#include "SDScreen.h"
#include "RecordScreen.h"




// NMEA 2000


bool analyze = false;
bool verbose = false;

tN2kDeviceList *pN2kDeviceList;

const unsigned long ReceiveMessages[] PROGMEM = {
    126208L, // Request, Command and "Reconocer?"
    126992L, // System Time
    127245L, // Rudder Angle
    127250L, // * Rhumb - Vessel heading
    127258L, // * Magnetic Variation
    128259L, // Speed over water
    129026L, // * Fast COG, SOG update
    129029L, // * Position GNSS (Date, time, lat, lon)
    129283L, // * XTE
    129284L, // * Route information
    129285L, // * Active Waypoint data
    130306L  // * Wind data
};

const tNMEA2000::tProductInformation AutopilotProductInformation PROGMEM = {
    1300,         // N2kVersion
    200,          // Manufacturer's product code
    "87180-2-EN", // Manufacturer's Model ID
    "3.1.2",      // Manufacturer's Software version code
    "LG-1",       // Manufacturer's Model version
    "00000001",   // Manufacturer's Model serial code
    1,            // CertificationLevel
    4             // LoadEquivalency
};

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char AutopilotManufacturerInformation[] PROGMEM = "RAYMARINE, INC., fgorina@gmail.com";
const char AutopilotInstallationDescription1[] PROGMEM = "https://www.azimutmarine.es/docs/manuales/Raymarine/RAY_Evolution%20EV100,%20ACUx_ESP.pdf";
const char AutopilotInstallationDescription2[] PROGMEM = "Test";

const unsigned long AutopilotSerialNumber PROGMEM = 1;
const unsigned char AutopilotDeviceFunction = 150; // Autopilot
const unsigned char AutopilotDeviceClass = 40;     // Steering and Control Surfaces
const uint16_t AutopilotManufacturerCode = 1851;   // Raymarine
const unsigned char AutopilotIndustryGroup = 4;    // Marine



// Global variables + State


static String wifi_ssid = "starlink_mini";          // Store the name of the wireless network.
static String wifi_password = "ailataN1991"; // Store the password of the wireless network.
static IPAddress signalk_tcp_host = IPAddress(192, 168, 1, 2);



long lastTime = millis();
static unsigned long last_message;
static unsigned long last_touched;

#define DISPLAY_ACTIVE 0
#define DISPLAY_SLEEPING 1
#define DISPLAY_WAKING 2

static int displaySaver = DISPLAY_ACTIVE;
static char buffer[64];

Screen *currentScreen = nullptr;
// Test variables

double speed = 5.0; // 5 knots
double heading = 0.0; // In radians


// State

tState *state{new tState()};

// Screens

Screen* screens[5] = {
   new MenuScreen(TFT_HOR_RES, TFT_VER_RES, "Logs"),
  new RecordScreen(TFT_HOR_RES, TFT_VER_RES, "Record", state, 10000),
  new SDScreen(TFT_HOR_RES, TFT_VER_RES, "Logs"),
  nullptr,
  nullptr

};

boolean startWiFi();


// WiFI
boolean checkConnection()
{                // Check wifi connection.
  int count = 0; // count.
  while (count < 30)
  { // If you fail to connect to wifi within 30*350ms (10.5s), return false; otherwise return true.
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }
    delay(350);
    count++;
  }
  return false;
}

boolean startWiFi()
{ // Check whether there is wifi configuration information storage, if there is return 1, if no return 0.

  // WiFi.setAutoConnect(true);

  Serial.println("Connecting to ");
  Serial.print(wifi_ssid);
  Serial.print(" ");
  Serial.println(wifi_password);
  WiFi.begin((char *)wifi_ssid.c_str(), (char *)wifi_password.c_str());
  if (checkConnection())
  {
    Serial.print("Connected to ");
    Serial.print(wifi_ssid);
    Serial.print(" IP ");
    Serial.println(WiFi.localIP());

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo) || timeinfo.tm_year < 20)
    {
      configTime(0, 0, "europe.pool.ntp.org");
      Serial.println("Synced RTC from NTP");
    }
    else
    {
      Serial.println("RTC already synced");
    }


    return true;
  }
  return false;
}


void HandleNMEA2000Msg(const tN2kMsg &N2kMsg){
  state->HandleNMEA2000Msg(N2kMsg, analyze, verbose);
}

void setup_NMEA2000()
{

  NMEA2000.SetProductInformation(&AutopilotProductInformation);
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(AutopilotManufacturerInformation, AutopilotInstallationDescription1, AutopilotInstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(AutopilotSerialNumber,     // Unique number. Use e.g. Serial number.
                                AutopilotDeviceFunction,   // Device function=Autopìlot. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                AutopilotDeviceClass,      // Device class=Steering and Control Surfaces. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                AutopilotManufacturerCode, // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                AutopilotIndustryGroup     // Industry Group
  );

  Serial.begin(115200);
  NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on theTransmitTransmit  bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 25);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);  ttttrtt   // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)

  //  NMEA2000.SetN2kCANMsgBufSize(2);                    // For this simple example, limit buffer size to 2, since we are only sending data
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.

  // NMEA2000.ExtendTransmitMessages(TransmitMessages); //We don't transmit messages

  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  // Set Group Handlers
  /*
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN65379(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN127250(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN127245(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN65360(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new tN2kGroupFunctionHandlerForPGN65345(&NMEA2000, &pypilot));
    NMEA2000.SetN2kSource(204);

    NMEA2000.SetOnOpen(OnN2kOpen);

    */
  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);
  NMEA2000.Open();
}

// Menu Management

void switchTo(int i){
  Screen* oldScreen = currentScreen;

    if(i >= 0 && i < 5){
      if (screens[i] != nullptr){
       if(oldScreen != nullptr){
         oldScreen->exit();
        }
        currentScreen = screens[i];
        currentScreen->enter();
      }else{
        Serial.print("Screen ");
        Serial.print(i);
        Serial.println(" not implemented");
      }
    }
}

void test_step(){
  unsigned long m = millis();
  time_t t = time(nullptr);
  if(M5.Touch.changed){
    heading = heading + 10.0 / 180.0 * PI;
  }
  if (m - last_message > 500){
    
    
    // Compute a distance. Speed = 5kt

    double distance = 5.0 * 1852.0 * ((m - last_message) / 3600000.0);  // distance in meters
    Serial.println(distance);
    // Compute a direction 
    double dlat = distance * cos(heading);
    double dlon = distance * sin(heading);

    // Compute a new position
    double lat = state->position.latitude + (dlat / 1852.0 / 60.0);
    double lon = state->position.longitude + (dlon / (1852.0 * cos(state->position.latitude / 180.0 * PI)) / 60.0);

    // Set neew position
    state->position.latitude = lat;
    state->position.longitude = lon;
    state->position.when = t;
    state->position.origin = 99;

    // Set the cog and sog

    state->cog.heading = heading;
    state->cog.when = time(nullptr);
    state->cog.origin = 99;

    state->heading.heading = heading;
    state->heading.when = t;
    state->heading.origin = 99;

    state->sog.value = speed / 3600 * 1852.0;
    state->sog.when = t;
    state->sog.origin = 99;

    state->wind.angle = (60.0 + random(-5, 5) )/ 180.0 * PI;
    state->wind.speed = 15.0 +random(-5, 5) * 1852.0 / 3600.0;
    state->wind.when = t;
    state->wind.origin = 99;
    state->wind.reference = tN2kWindReference::N2kWind_Apparent;

    last_message = m;
    
  }


}
void setup()
{

  M5.begin();
  Serial.begin(115200);
  SD.begin();
  M5.Lcd.setFreeFont(&unicode_24px);
  setup_NMEA2000();

  last_touched = millis();

  //M5.Lcd.wakeup();
  Serial.println("Entering Menu Screen");
  
  //currentScreen = new MenuScreen(TFT_HOR_RES, TFT_VER_RES, "Logs");
  currentScreen = screens[0];
  currentScreen->enter();

}
void loopTask()
{

  if(currentScreen != nullptr){
    int newScreen = currentScreen->run();

    if (newScreen >= 0 && newScreen < 5){
      Serial.println("About to switch");
      switchTo(newScreen);
    }
  }
  
}

#define GO_SLEEP_TIMEOUT 60000ul // 5 '


void loop()
{
  //NMEA2000.ParseMessages();
  

  if (!checkConnection())
  {
    startWiFi();
  }
  M5.update();
  test_step();
  
  loopTask();
  delay(5);
}