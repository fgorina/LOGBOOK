#define TFT_HOR_RES 320
#define TFT_VER_RES 240

#define ESP32_CAN_TX_PIN GPIO_NUM_17
#define ESP32_CAN_RX_PIN GPIO_NUM_18

#include "N2kDeviceList.h"
#include "esp_task_wdt.h"
#include "net_nmea0183.h"
#include "net_signalk.h"
#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include <M5Unified.h>
#include <N2kMessages.h>
#include "NMEA2000_twai.h"
#include "TwaiLog.h"
tNMEA2000 &NMEA2000 = *(new tNMEA2000_twai(ESP32_CAN_TX_PIN, ESP32_CAN_RX_PIN));
#include <Preferences.h>
#include <time.h>

#include "PyTypes.h"

#include <ESPmDNS.h>
#include "BuildInfo.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "State.h"

#include <SD.h>

#include <FS.h>

#include "LogWebServer.h"
#include "Utils.h"
// Screens

#include "InfoScreen.h"
#include "MenuScreen.h"
#include "N2KDevices.h"
#include "RecordScreen.h"
#include "SDScreen.h"
#include "WaitScreen.h"

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

const tNMEA2000::tProductInformation LogProductInformation PROGMEM = {
    1300,       // N2kVersion
    201,        // Manufacturer's product code
    "LOG-001",  // Manufacturer's Model ID
    "0.0.1",    // Manufacturer's Software version code
    "LOG-001",  // Manufacturer's Model version
    "00000001", // Manufacturer's Model serial code
    1,          // CertificationLevel
    4           // LoadEquivalency
};

// ---  Example of using PROGMEM to hold Configuration information.  However,
// doing this will prevent any updating of
//      these details outside of recompiling the program.
const char LogManufacturerInformation[] PROGMEM =
    "Paco Gorina, fgorina@gmail.com";
const char LogInstallationDescription1[] PROGMEM =
    "Just connect and configure with a web browser";
const char LogInstallationDescription2[] PROGMEM =
    "Select NMEA 2000, SignalK and WiFi and format settings";

// Unique per installation. 0 means "not yet generated"; readPreferences()
// fills this in on first boot and persists it (see deviceName for the same
// pattern). Must fit NMEA2000's 21-bit unique number field (< 2097152).
static unsigned long n2kSerialNumber = 0;
const unsigned char LogDeviceFunction PROGMEM = 140; // Log Recorder
const unsigned char LogtDeviceClass = 20;            // Safety Systems
const uint16_t LogManufacturerCode = 2046;           // Free?
const unsigned char LogIndustryGroup = 4;            // Marine

// Global variables + State

#ifdef DEV
String wifi_ssid =
    "elrond"; //"TP-LINK_2695";//"Yamato"; //"starlink_mini";   // Store the
              //name of the wireless network.
String wifi_password =
    "ailataN1991"; // "39338518"; //ailataN1991"; // Store the password of the
                   // wireless network.
String skServer = "192.168.001.150"; //"192.168.1.54";
int skPort = 3000;
bool useN2k = false;
bool useSK = true;
bool use0183 = false;
#else
String wifi_ssid =
    "Yamato"; //"TP-LINK_2695";//"Yamato"; //"starlink_mini";   // Store the
              //name of the wireless network.
String wifi_password =
    "ailataN1991"; // "39338518"; //ailataN1991"; // Store the password of the
                   // wireless network.
String skServer = "192.168.1.2"; //"192.168.1.54";
int skPort = 3000;
bool useN2k = true;
bool useSK = false;
bool use0183 = false;
#endif

String n2kSources = "15";
int sources[MAX_SOURCES] = {15, 100, 0, 0, 0, 0, 0, 0, 0, 0,
                            0,  0,   0, 0, 0, 0, 0, 0, 0, 0};
int n_sources = 1;

// static IPAddress signalk_tcp_host = IPAddress(192,168,1,204);
// //IPAddress(192, 168, 1, 2);

bool starting = true; // Is true if WiFi is not configured
String deviceName;    // Unique AP SSID, e.g. "LOGBOOK_427"

long lastTime = millis();
static unsigned long last_message;
static unsigned long last_touched;

static char buffer[64];

Screen *currentScreen = nullptr;

// Preferences
Preferences preferences;
void writePreferences();
void readPreferences();
void lookupPypilot();

// SD mutex — acquire before any SD card access
SemaphoreHandle_t sdMutex;

// Test variables

double speed = 5.0;   // 5 knots
double heading = 0.0; // In radians

// State

tState *state{new tState()};

String myIp = "Connecting...";

// SignalK server

NetSignalkWS *skWsServer = new NetSignalkWS(skServer.c_str(), skPort, state);
NetNMEA0183 *nmea0183 = new NetNMEA0183(skServer.c_str(), 10110, state);

// Screens

Screen *screens[6] = {
    new MenuScreen(state, TFT_HOR_RES, TFT_VER_RES, "Logs"),
    new RecordScreen(TFT_HOR_RES, TFT_VER_RES, "Record", state, 1000),
    new SDScreen(TFT_HOR_RES, TFT_VER_RES, "Logs", state),
    new InfoScreen(&deviceName, &wifi_ssid, &myIp, &useN2k, &useSK, &use0183,
                   &skServer, &skPort, &n2kSources, TFT_HOR_RES, TFT_VER_RES,
                   "Info"),
    nullptr, // N2KDevices to be defined in setupN2K
    nullptr,
};

boolean startWiFi();
void switchTo(int i);

// ── Moving filter (EMA on velocity vector) ───────────────────────────────────
// Same algorithm and constants as Tab5Nav.
// Runs every second in loop(); auto-starts recording when the boat starts
// moving (switches to RecordScreen) and stops when it becomes stationary.
static float mf_fastX = 0, mf_fastY = 0;
static float mf_slowX = 0, mf_slowY = 0;
static bool mf_moving = false;
static bool mf_initialized = false;
static unsigned long mf_lastUpdate = 0;

static constexpr float MF_START_THRESH = 0.26f; // m/s, fast EMA → start
static constexpr float MF_STOP_THRESH = 0.15f;  // m/s, slow EMA → stop
static constexpr float MF_ALPHA_FAST = 0.0056f;
static constexpr float MF_ALPHA_SLOW = 0.004f; // τ ≈ 5 min

void updateMovingFilter() {
  if (millis() - mf_lastUpdate < 1000)
    return;
  mf_lastUpdate = millis();

  // Treat stale SOG as 0 — data source went silent
  float sog =
      (time(nullptr) - state->sog.when <= 10) ? (float)state->sog.value : 0.0f;
  // COG is undefined at zero speed — use 0 (velocity vector is zero regardless)
  float cog = (!isnan(state->cog.heading)) ? state->cog.heading : 0.0f;

  float vx = sog * sinf(cog);
  float vy = sog * cosf(cog);

  if (!mf_initialized) {
    mf_fastX = mf_slowX = vx;
    mf_fastY = mf_slowY = vy;
    mf_initialized = true;
  } else {
    mf_fastX += MF_ALPHA_FAST * (vx - mf_fastX);
    mf_fastY += MF_ALPHA_FAST * (vy - mf_fastY);
    mf_slowX += MF_ALPHA_SLOW * (vx - mf_slowX);
    mf_slowY += MF_ALPHA_SLOW * (vy - mf_slowY);
  }

  float fastMag = sqrtf(mf_fastX * mf_fastX + mf_fastY * mf_fastY);
  float slowMag = sqrtf(mf_slowX * mf_slowX + mf_slowY * mf_slowY);

  bool wasMoving = mf_moving;
  if (!mf_moving && fastMag >= MF_START_THRESH)
    mf_moving = true;
  else if (mf_moving && slowMag < MF_STOP_THRESH)
    mf_moving = false;

  if (!wasMoving && mf_moving && currentScreen == screens[0]) {
    Serial.println("MovingFilter: started moving → switching to RecordScreen");
    switchTo(1);
  } else if (wasMoving && !mf_moving && currentScreen == screens[1]) {
    Serial.println("MovingFilter: stopped moving → switching to MenuScreen");
    mf_initialized = false; // reset so stale EMA can't immediately re-trigger
    switchTo(0);
  }
}

// Preferences

void writePreferences() {
  preferences.begin("Logbook", false);
  preferences.remove("SSID");
  preferences.remove("PASSWD");
  preferences.remove("PPHOST");
  preferences.remove("PPPORT");
  preferences.remove("FILEFORMAT");
  preferences.remove("USEN2K");
  preferences.remove("USESK");
  preferences.remove("USE0183");

  preferences.putString("SSID", wifi_ssid);
  preferences.putString("PASSWD", wifi_password);
  preferences.putString("PPHOST", skServer);
  preferences.putInt("PPPORT", skPort);
  preferences.putBool("FILEFORMAT", ((RecordScreen *)screens[1])->xmlFormat);
  preferences.putBool("USEN2K", useN2k);
  preferences.putBool("USESK", useSK);
  preferences.putBool("USE0183", use0183);
  n2kSources = join(sources, MAX_SOURCES, ',');
  preferences.putString("N2KSOURCES", n2kSources);
  preferences.putString("DEVICENAME", deviceName);
  preferences.putULong("N2KSERIAL", n2kSerialNumber);
  preferences.putUShort("SAILSMASK", state->sailsAvailable);
  preferences.putString("SAILS", join(state->sails, tState::N_SAILS, ','));
  preferences.end();
}

// Called by SailsScreen on every change so the active sails survive a reboot
void writeSailState() {
  preferences.begin("Logbook", false);
  preferences.putString("SAILS", join(state->sails, tState::N_SAILS, ','));
  preferences.end();
}

void readPreferences() {
  preferences.begin("Logbook", true);
  wifi_ssid = preferences.getString("SSID", wifi_ssid);
  wifi_password = preferences.getString("PASSWD", wifi_password);
  skServer = preferences.getString("PPHOST", skServer);
  skPort = preferences.getInt("PPPORT", skPort);
  ((RecordScreen *)screens[1])->xmlFormat =
      preferences.getBool("FILEFORMAT", false);

  useN2k = preferences.getBool("USEN2K", false);
  useSK = preferences.getBool("USESK", false);
  use0183 = preferences.getBool("USE0183", false);

  deviceName = preferences.getString("DEVICENAME", "");
  n2kSerialNumber = preferences.getULong("N2KSERIAL", 0);
  state->sailsAvailable = preferences.getUShort("SAILSMASK", 0);
  String sailState = preferences.getString("SAILS", "");
  splitter((char *)(sailState.c_str()), state->sails, ',', sailState.length(),
           tState::N_SAILS);
  n2kSources = preferences.getString("N2KSOURCES", n2kSources);
  n_sources = splitter((char *)(n2kSources.c_str()), sources, ',',
                       n2kSources.length(), MAX_SOURCES);
  for (int i = n_sources; i < MAX_SOURCES; i++) {
    sources[i] = -1;
  }

  preferences.end();

  if (deviceName.isEmpty()) {
    // First boot: generate a unique name and persist it so the same
    // name survives reboots but differs from every other device.
    deviceName = "LOGBOOK_" + String(random(100, 1000));
    preferences.begin("Logbook", false);
    preferences.putString("DEVICENAME", deviceName);
    preferences.end();
  }

  if (n2kSerialNumber == 0) {
    // First boot: generate a unique NMEA2000 device serial number and
    // persist it, so every installation claims a distinct NAME on the bus
    // without needing a per-board firmware edit. Range stays within
    // NMEA2000's 21-bit unique number field.
    n2kSerialNumber = random(1000, 2000000);
    preferences.begin("Logbook", false);
    preferences.putULong("N2KSERIAL", n2kSerialNumber);
    preferences.end();
  }
  Serial.println("============== Preferences ================= ");
  Serial.print("ssid : ");
  Serial.println(wifi_ssid);
  Serial.print("password : ");
  Serial.println(wifi_password);
  Serial.print("skServer : ");
  Serial.println(skServer);
  Serial.print("skPort : ");
  Serial.println(skPort);
  Serial.print("xmlFormat : ");
  Serial.println(((RecordScreen *)screens[1])->xmlFormat);
  Serial.print("use N2k : ");
  Serial.println(useN2k);
  Serial.print("use SignalK : ");
  Serial.println(useSK);
  Serial.print("use NMEA0183: ");
  Serial.println(use0183);
  Serial.print("Sources : ");
  for (int i = 0; i < MAX_SOURCES; i++) {
    if (sources[i] >= 0) {
      Serial.print(sources[i]);
      Serial.print(",");
    }
  }
  Serial.println();
  Serial.println("============================================ ");
}
// WiFI
boolean checkConnection() { // Check wifi connection.
  int count = 0;            // count.
  while (count < 1000) {    // If you fail to connect to wifi within 30*350ms
                            // (10.5s), return false; otherwise return true.
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
    delay(10);
    count++;
  }
  return false;
}

boolean startWiFiAP() {
  Serial.println("Creating wifi AP: " + deviceName + " / " AP_PASSWORD);
  WiFi.mode(wifi_mode_t::WIFI_MODE_AP);
  WiFi.softAP(deviceName.c_str(), AP_PASSWORD);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("Ip : " + IP.toString());
  myIp = IP.toString();

  // Start mdns so we have a name

  if (!MDNS.begin(deviceName.c_str())) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    Serial.println("mDNS responder started");
  }

  startWebServer();
  starting = false;
  return true;
}
boolean startWiFi() { // Check whether there is wifi configuration information
                      // storage, if there is return 1, if no return 0.

  // WiFi.setAutoConnect(true);

  Serial.println("Connecting to ");
  Serial.print(wifi_ssid);
  Serial.print(" ");
  Serial.println(wifi_password);
  WiFi.mode(wifi_mode_t::WIFI_MODE_STA);
  // Modem sleep drops multicast packets, so mDNS queries get lost and the
  // client has to retry (seconds) before <deviceName>.local resolves.
  WiFi.setSleep(false);
  WiFi.begin((char *)wifi_ssid.c_str(), (char *)wifi_password.c_str());

  if (checkConnection()) {
    Serial.print("Connected to ");
    Serial.print(wifi_ssid);
    Serial.print(" IP ");
    Serial.println(WiFi.localIP());
    myIp = WiFi.localIP().toString();

    configTime(0, 0, "europe.pool.ntp.org");
    Serial.println("Syncing RTC from NTP");
    {
      struct tm ntpInfo;
      int tries = 0;
      while (!getLocalTime(&ntpInfo, 1000) && tries < 10) {
        Serial.printf("NTP sync attempt %d/10...\n", ++tries);
      }
      if (tries < 10) {
        Serial.println("NTP sync OK");
      } else {
        Serial.println("NTP sync failed — will use GPS time");
      }
    }
    // Start mdns so we have a name

    if (!MDNS.begin(deviceName.c_str())) {
      Serial.println("Error setting up MDNS responder!");
    } else {
      Serial.println("mDNS responder started");
    }
    // Try to connect to signalk
    vTaskDelay(5);
    if (skServer.length() > 0 && skPort > 0 && useSK) {
      skWsServer->begin(); // Connect to the SignalK TCP server
    }
    vTaskDelay(5);
    // Now start Web Server
    startWebServer();
    vTaskDelay(5);
    currentScreen->draw();
    return true;
  }
  return false;
}

// Returns true if source is in sources
bool checkSource(unsigned char source) {
  if (n_sources == 0) {
    return true;
  }

  for (int i = 0; i < n_sources; i++) {
    if (sources[i] == source) {
      return true;
    }
  }
  return false;
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  if (checkSource(N2kMsg.Source)) {
    state->HandleNMEA2000Msg(N2kMsg, analyze, verbose);
  }
}

void setup_NMEA2000() {

  NMEA2000.SetProductInformation(&LogProductInformation);
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(LogManufacturerInformation,
                                              LogInstallationDescription1,
                                              LogInstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(
      n2kSerialNumber, // Unique number, generated per installation on first
                       // boot and persisted (see readPreferences).
      LogDeviceFunction,     // Device function=Autopìlot. See codes on
                         // https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      LogtDeviceClass, // Device class=Steering and Control Surfaces. See codes
                       // on
                       // https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      LogManufacturerCode, // Just choosen free from code list on
                           // https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
      LogIndustryGroup     // Industry Group
  );

  NMEA2000.SetForwardStream(&TwaiLogger);
  NMEA2000.SetForwardType(
      tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for
                             // default Actisense format.

  // If you also want to see all traffic on theTransmitTransmit  bus use
  // N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 25);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);  ttttrtt   // Uncomment
  // this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(true); // Disable all msg forwarding to USB (=Serial)

  //  NMEA2000.SetN2kCANMsgBufSize(2);                    // For this simple
  //  example, limit buffer size to 2, since we are only sending data
  // Define OnOpen call back. This will be called, when CAN is open and system
  // starts address claiming.

  // NMEA2000.ExtendTransmitMessages(TransmitMessages); //We don't transmit
  // messages

  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  // Set Group Handlers
  /*
    NMEA2000.AddGroupFunctionHandler(new
    tN2kGroupFunctionHandlerForPGN65379(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new
    tN2kGroupFunctionHandlerForPGN127250(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new
    tN2kGroupFunctionHandlerForPGN127245(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new
    tN2kGroupFunctionHandlerForPGN65360(&NMEA2000, &pypilot));
    NMEA2000.AddGroupFunctionHandler(new
    tN2kGroupFunctionHandlerForPGN65345(&NMEA2000, &pypilot));
    NMEA2000.SetN2kSource(204);

    NMEA2000.SetOnOpen(OnN2kOpen);

    */
  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);
  screens[4] =
      new N2KDevices(pN2kDeviceList, TFT_HOR_RES, TFT_VER_RES, "Devices");
  NMEA2000.Open();
}

// Menu Management

void switchTo(int i) {
  Screen *oldScreen = currentScreen;

  // If the display is sleeping when a screen switch happens (e.g. recording
  // stopped while the saver was active), wake it up so the new screen is
  // visible.
  if (state->displaySaver != DISPLAY_ACTIVE) {
    M5.Display.wakeup();
    M5.Display.setBrightness(128);
    state->displaySaver = DISPLAY_ACTIVE;
    last_touched = millis();
  }

  if (i >= 0 && i < 5) {
    if (screens[i] != nullptr) {
      if (oldScreen != nullptr) {
        oldScreen->exit();
      }
      currentScreen = screens[i];
      currentScreen->enter();
    } else {
      Serial.print("Screen ");
      Serial.print(i);
      Serial.println(" not implemented");
    }
  }
}
/// Tasks

TaskHandle_t taskNetwork;
TaskHandle_t taskN2K;
TaskHandle_t taskWss;
TaskHandle_t task0183;

void networkTask(void *parameter) {

  while (true) {
    // Check wifi_ssid first (cheap). In AP mode wifi_ssid is empty so
    // checkConnection() — which blocks up to 10 s waiting for WL_CONNECTED —
    // is never called.
    if (!wifi_ssid.isEmpty() && !checkConnection()) {
      Serial.println("Starting WiFi");
      startWiFi();
    }

    handleWebServer();

    // 10 ms yield is enough for interactive use (100 callbacks/sec).
    // A tighter loop hammers the lwIP core mutex, blocking the WiFi driver's
    // WPA2 handshake and making AP-mode connections appear to fail.
    vTaskDelay(10);
  }
}

void n2KTask(void *parameter) {
  while (true) {
    if (useN2k) {
      NMEA2000.ParseMessages();
    }
    vTaskDelay(10);
  }
}

void wssTask(void *parameter) {
  while (true) {
    if (useSK && checkConnection()) {
      skWsServer->run();
    }
    vTaskDelay(10);
  }
}

void nmea0183Task(void *parameter) {
  while (true) {
    if (use0183 && checkConnection()) {
      nmea0183->run();
    }
    vTaskDelay(20);
  }
}
void uiTask(const m5::touch_detail_t &t) {

  if (currentScreen != nullptr) {
    int newScreen = currentScreen->run(t);

    if (newScreen >= 0 && newScreen < 5) {
      Serial.println("About to switch");
      switchTo(newScreen);
    }
  }
}

void resetNetwork() {
  // Sets preferences to work as STA
  // with ssig "logbook"
  // and passwd "12345678"
  // No SK
  // No N2k

  wifi_ssid = "";
  wifi_password = "";
  skServer = "";
  skPort = 0;
  useN2k = false;
  useSK = false;

  writePreferences();

  ESP.restart();
}

void splash() {
  M5.Display.clear();
  unsigned long s = millis();
  long press = -1;
  const unsigned long duration = 10000;

  M5.Display.setFont(&fonts::FreeSans9pt7b);
  // We loop 10 seconds, just waiting for a network reset
  M5.Display.setTextDatum(TC_DATUM);
  M5.Display.drawString("Logbook", TFT_HOR_RES / 2, 10);

  M5.Display.setTextDatum(CL_DATUM);
  M5.Display.drawString("SSID: " + wifi_ssid, 10, 50);
  M5.Display.drawString("SK Server: " + skServer + ":" + skPort, 10, 90);
  M5.Display.drawString("Use Nemea 2000: " + String(useN2k ? "Si" : "No"), 10,
                        130);
  M5.Display.drawString("Use SignalK: " + String(useSK ? "Si" : "No"), 10, 170);

  M5.Display.setTextDatum(CC_DATUM);
  M5.Display.drawString("Toqueu per reset", TFT_HOR_RES / 2, 200);

  while (millis() - s < duration) {
    M5.update();
    auto count = M5.Touch.getCount();
    if (count > 0) {
      auto t = M5.Touch.getDetail(0);
      if (t.wasPressed()) {
        press = millis();

      } else if (t.wasReleased()) {
        if (millis() - press > 1000) {
          Serial.println("Resetting Network");
          resetNetwork();
        } else {
          press = -1;
        }
      }
    }
    delay(1);
  }
  return;
}

void setup() {

  M5.begin();
  M5.Display.setRotation(1); // 3 per la versio NMEA 2000 del M5Though, 1 for the rest
  Serial.begin(115200);
  Serial.println("Firmware: " FW_VERSION " (" FW_BUILD_TIME ")");
  M5.Display.wakeup();

  readPreferences();
  if (!wifi_ssid.isEmpty()) {
    splash();
  } else {
    startWiFiAP();
  }

  M5.Display.setFont(&fonts::FreeSans12pt7b);
  M5.Display.setTextSize(1.0);
  sdMutex = xSemaphoreCreateMutex();

  // CoreS3 SE: SCK=G36, MISO=G35, MOSI=G37, CS=G4
  SPI.begin(36, 35, 37, 4);
  if (!SD.begin(4, SPI, 25000000)) {
    Serial.println("SD card mount failed");
  } else {
    Serial.println("SD card mounted");
  }

  if (useN2k) {
    // TwaiLogger.begin(); // TWAI confirmed working, logging disabled
    setup_NMEA2000();
  }

  // M5.Lcd.wakeup();
  Serial.println("Starting Tasks");
  // 16 KB stack: the web-server call chain (handleClient → handler → SD/FATFS)
  // overflows a 4 KB stack, causing silent hangs on any SD access.
  xTaskCreate(networkTask, "NetworkTask", 16384, NULL, 1, &taskNetwork);
  Serial.println("Network Task Created");
  if (useN2k) {
    xTaskCreate(n2KTask, "N2kTask", 4000, NULL, 0, &taskN2K);
    Serial.println("N2K Task Created");
  }

  if (useSK) {
    xTaskCreate(wssTask, "WSS Task", 4000, NULL, 0, &taskWss);
    Serial.println("WSS Task Created");
  }

  if (use0183) {
    xTaskCreate(nmea0183Task, "NMEA0183 Task", 8192, NULL, 0, &task0183);
    Serial.println("NMEA0183 Task Created");
  }

  // currentScreen = new MenuScreen(TFT_HOR_RES, TFT_VER_RES, "Logs");

  Serial.println("Opening main screen");
  if (wifi_ssid.isEmpty()) {
    currentScreen = screens[3];
  } else {
    currentScreen = screens[0];
  }
  currentScreen->enter();
  last_touched = millis();
}

#define GO_SLEEP_TIMEOUT 30000ul // 5 '

void *p;

void loop() {
  M5.update();
  auto &t = M5.Touch.getDetail(0);
  updateMovingFilter();
  uiTask(t);

  auto count = M5.Touch.getCount();
  if (count > 0) {
    if (t.wasPressed() || t.wasReleased()) {
      last_touched = millis();
      Serial.println("Touched");
      if (state->displaySaver == DISPLAY_SLEEPING && t.wasPressed()) {
        Serial.println("Waking Up");
        M5.Display.wakeup();
        M5.Display.setBrightness(128);
        state->displaySaver = DISPLAY_WAKING;
      } else if (state->displaySaver == DISPLAY_WAKING && t.wasReleased()) {
        Serial.println("Activating");
        state->displaySaver = DISPLAY_ACTIVE;
        last_touched = millis();
        if (currentScreen != nullptr)
          currentScreen->draw(); // Refresh after wakeup
      }
    }
  }

  if (millis() - last_touched > GO_SLEEP_TIMEOUT &&
      state->displaySaver == DISPLAY_ACTIVE) {
    Serial.println("Going to Sleep");
    M5.Display.sleep();
    M5.Display.setBrightness(0);
    state->displaySaver = DISPLAY_SLEEPING;
  }

  vTaskDelay(50);
}