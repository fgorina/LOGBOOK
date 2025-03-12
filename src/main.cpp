#define TFT_HOR_RES 320
#define TFT_VER_RES 240

#define ESP32_CAN_TX_PIN GPIO_NUM_19
#define ESP32_CAN_RX_PIN GPIO_NUM_27

#include <Arduino.h>
#include <WebServer.h>
#include <M5Tough.h>
#include <CUF_24.h>
#include <time.h>
#include <Preferences.h>
#include <NMEA2000_esp32.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include "N2kDeviceList.h"
#include <ArduinoWebsockets.h>
#include "net_signalk.h"

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

const tNMEA2000::tProductInformation LogProductInformation PROGMEM = {
    1300,         // N2kVersion
    201,          // Manufacturer's product code
    "LOG-001", // Manufacturer's Model ID
    "0.0.1",      // Manufacturer's Software version code
    "LOG-001",       // Manufacturer's Model version
    "00000001",   // Manufacturer's Model serial code
    1,            // CertificationLevel
    4             // LoadEquivalency
};

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char LogManufacturerInformation[] PROGMEM = "Paco Gorina, fgorina@gmail.com";
const char LogInstallationDescription1[] PROGMEM = "Just connect and configure with a web browser";
const char LogInstallationDescription2[] PROGMEM = "Select NMEA 2000, SignalK and WiFi and format settings";

const unsigned long AutopilotSerialNumber PROGMEM = 13;
const unsigned char LogDeviceFunction PROGMEM = 140; // Log Recorder
const unsigned char LogtDeviceClass = 20;     // Safety Systems
const uint16_t LogManufacturerCode = 2046;   // Free?
const unsigned char LogIndustryGroup = 4;    // Marine

// Global variables + State

#define DEV

#ifdef DEV
static String wifi_ssid = "elrond";          //"TP-LINK_2695";//"Yamato"; //"starlink_mini";   // Store the name of the wireless network.
static String wifi_password = "ailataN1991"; // "39338518"; //ailataN1991"; // Store the password of the wireless network.
static String skServer = "192.168.001.150";  //"192.168.1.54";
int skPort = 3000;
#else
static String wifi_ssid = "Yamato";          //"TP-LINK_2695";//"Yamato"; //"starlink_mini";   // Store the name of the wireless network.
static String wifi_password = "ailataN1991"; // "39338518"; //ailataN1991"; // Store the password of the wireless network.
const char *skServer = "192.168.1.2";        //"192.168.1.54";
const int skPort = 3000;
#endif

// static IPAddress signalk_tcp_host = IPAddress(192,168,1,204); //IPAddress(192, 168, 1, 2);

// Http Server
WebServer server(80);

long lastTime = millis();
static unsigned long last_message;
static unsigned long last_touched;

#define DISPLAY_ACTIVE 0
#define DISPLAY_SLEEPING 1
#define DISPLAY_WAKING 2

static int displaySaver = DISPLAY_ACTIVE;
static char buffer[64];

Screen *currentScreen = nullptr;

// Preferences
Preferences preferences;
void writePreferences();
void readPreferences();
void lookupPypilot();

// Test variables

double speed = 5.0;   // 5 knots
double heading = 0.0; // In radians

// State

tState *state{new tState()};

String myIp = "Connecting...";

// SignalK server

NetSignalkWS *skWsServer = new NetSignalkWS(skServer.c_str(), skPort, state);

// Screens

Screen *screens[4] = {
    new MenuScreen(TFT_HOR_RES, TFT_VER_RES, "Logs"),
    new RecordScreen(TFT_HOR_RES, TFT_VER_RES, "Record", state, 1000),
    new SDScreen(TFT_HOR_RES, TFT_VER_RES, "Logs"),
    nullptr

};

boolean startWiFi();

// Preferences

void writePreferences()
{
  preferences.begin("Logbook", false);
  preferences.remove("SSID");
  preferences.remove("PASSWD");
  preferences.remove("PPHOST");
  preferences.remove("PPPORT");
  preferences.remove("FILEFORMAT");

  preferences.putString("SSID", wifi_ssid);
  preferences.putString("PASSWD", wifi_password);
  preferences.putString("PPHOST", skServer);
  preferences.putInt("PPPORT", skPort);
  preferences.putBool("FILEFORMAT", ((RecordScreen *)screens[1])->xmlFormat);
  preferences.end();
}

void readPreferences()
{
  preferences.begin("Logbook", true);
  wifi_ssid = preferences.getString("SSID", wifi_ssid);
  wifi_password = preferences.getString("PASSWD", wifi_password);
  skServer = preferences.getString("PPHOST", skServer);
  skPort = preferences.getInt("PPPORT", skPort);
  ((RecordScreen *)screens[1])->xmlFormat = preferences.getBool("FILEFORMAT", false);
  preferences.end();
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
  Serial.println("============================================ ");
}
// Web server handlers

String getContentType(String filename)
{
  if (server.hasArg("download"))
  {
    return "application/octet-stream";
  }
  else if (filename.endsWith(".htm"))
  {
    return "text/html";
  }
  else if (filename.endsWith(".html"))
  {
    return "text/html";
  }
  else if (filename.endsWith(".css"))
  {
    return "text/css";
  }
  else if (filename.endsWith(".js"))
  {
    return "application/javascript";
  }
  else if (filename.endsWith(".png"))
  {
    return "image/png";
  }
  else if (filename.endsWith(".gif"))
  {
    return "image/gif";
  }
  else if (filename.endsWith(".jpg"))
  {
    return "image/jpeg";
  }
  else if (filename.endsWith(".ico"))
  {
    return "image/x-icon";
  }
  else if (filename.endsWith(".xml"))
  {
    return "text/xml";
  }
  else if (filename.endsWith(".pdf"))
  {
    return "application/x-pdf";
  }
  else if (filename.endsWith(".zip"))
  {
    return "application/x-zip";
  }
  else if (filename.endsWith(".gz"))
  {
    return "application/x-gzip";
  }
  else if (filename.endsWith(".gpx"))
  {
    return "application/gpx+xml";
  }
  else if (filename.endsWith(".csv"))
  {
    return "text/plain";
  }
  return "text/plain";
}

bool handleFileRead(String path)
{

  if (path.endsWith("/"))
  {
    path += "index.htm";
  }

  if (!SD.exists(path))
  {
    Serial.println("File " + path + " not found.");
    server.send(404, "text/plain", "FileNotFound");
    return false;
  }

  Serial.println("handleFileRead: " + path);
  String contentType = getContentType(path);

  File file = SD.open(path, FILE_READ);
  if (file)
  {
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  else
  {
    Serial.println("File " + path + " not found.");
    return false;
  }
}

void handleFileList()
{
  Serial.println("handleFileList");

  File root = SD.open("/");

  String output = "<htlm><head><title>Logs</title></head><body>\n";
  output += "<h1><a href=\"/\">Logbook</a>/Logs</h1>\n";
  output += "<a href=\"/ask\">Esborrar tots els Logs</a></br>";
  output += "<ul>\n";
  if (root.isDirectory())
  {
    File file = root.openNextFile();
    while (file)
    {
      if (file.name()[0] != '.')
      {
        output += String("<li><a href=\"") + file.path() + String("\">") + file.name() + "</a>    <a href=del/" + file.name() + ">Delete</a></li>\n";
      }

      file = root.openNextFile();
    }
  }

  output += "</ul>\n";
  unsigned long len = output.length();
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);
}

void handleMenu()
{

  String output = "<html><head><title>Logbook by Paco Gorina</title></head><body>";
  output += "<h1>Logbook by Paco Gorina</h1>";
  output += "<ul>";
  output += "<li><a href=\"/prefs\">Prefer&egrave;ncies</a></li>";
  output += "<li><a href=\"/logs\">Logs</a></li>";

  output += "</ul>";
  output += "</body></html>";
  unsigned long len = output.length();
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);

}

void handleAskForDelete()
{
  Serial.println("handleAskForDelete");
  String output = "<html><head><title>Confirmeu, si us plau</title></head><body>";
  output += "Segur que voleu esborrar tots els logs? <a href=/clear>Si</a> <a href=/>No</a>";
  unsigned long len = output.length();
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);

}
void handleDeleteAll()
{
  Serial.println("handleDeleteAll");

  File root = SD.open("/");

  String output = "<htlm><head><meta http-equiv=\"refresh\" content=\"0;url=/\"><title>Logs</title></head><body>\n";
  output += "<h1>Logs</h1>\n";
  output += "<ul>\n";
  if (root.isDirectory())
  {
    File file = root.openNextFile();
    while (file)
    {
      if (file.name()[0] != '.')
      {
        SD.remove(file.path());
      }

      file = root.openNextFile();
    }
  }

  output += "</ul>\n";
  server.send(200, "text/html", output);
}

void deleteFile(String uri)
{
  Serial.println("deleteFile uri " + uri);
  String path = uri.substring(4, uri.length());
  Serial.println("deleteFile " + path);
  SD.remove(path);
  handleFileList();
}

void handlePreferences()
{
  Serial.println("handlePreferences");
  String output = "<html><head><title>Prefer&egrave;ncies</title></head><body>";
  output += "<h1><a href=\"/\">Logbook</a>/Prefer&egrave;ncies</h1>";
  output += "<form action=\"/updatePrefs\" method=\"post\">";
  output += "<table border=0>";
  output += "<tr><td><label for=\"ssid\">SSID:</label></td><td><input type=\"text\" id=\"ssid\" name=\"ssid\" value=\"" + wifi_ssid + "\"></td></tr>";
  output += "<tr><td><label for=\"password\">Password:</label></td><td><input type=\"password\" id=\"password\" name=\"password\" value=\"" + wifi_password + "\"></td></tr>";
  output += "<tr><td><label for=\"skserver\">SignalK Server:</label></td><td><input type=\"text\" id=\"skserver\" name=\"skserver\" value=\"" + skServer + "\"></td></tr>";
  output += "<tr><td><label for=\"skport\">SignalK Port:</label></td><td><input type=\"number\" id=\"skport\" name=\"skport\" value=\"" + String(skPort) + "\"></td></tr>";
  output += "<tr><td><label for=\"usexml\">Use GPX:</label></td><td><input type=\"checkbox\" id=\"usexml\" name=\"usexml\" value=\"on\" " + String(((RecordScreen *)screens[1])->xmlFormat ? "checked" : "") + "></td></tr>";
  output += "<tr><td colspan=2 align=center><input type=\"submit\" value=\"Submit\"></td></tr>";
  output += "</table>";
  output += "</form>";
  output += "</body></html>";
  unsigned long len = output.length();
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);
}

void handleUpdatePreferences()
{

  Serial.println("handleUpdatePreferences");
  if (server.hasArg("ssid"))
  {
    wifi_ssid = server.arg("ssid");
  }
  if (server.hasArg("password"))
  {
    wifi_password = server.arg("password");
  }
  if (server.hasArg("skserver"))
  {
    skServer = server.arg("skserver");
  }
  if (server.hasArg("skport"))
  {
    skPort = server.arg("skport").toInt();
  }
  if (server.hasArg("usexml"))
  {
    ((RecordScreen *)screens[1])->xmlFormat = server.arg("usexml") == "on";
  }
  else
  {
    ((RecordScreen *)screens[1])->xmlFormat = false;
  }

  writePreferences();
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}
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

void startWebServer()
{
  server.on("/", HTTP_GET, handleMenu);
  server.on("/ask", HTTP_GET, handleAskForDelete);
  server.on("/clear", HTTP_GET, handleDeleteAll);
  server.on("/prefs", HTTP_GET, handlePreferences);
  server.on("/logs", HTTP_GET, handleFileList);
  server.on("/updatePrefs", HTTP_POST, handleUpdatePreferences);

  server.onNotFound([]()
                    {
                      if (server.uri().startsWith("/del"))
                      {
                        deleteFile(server.uri());
                      }
                     
    else if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "FileNotFound");
    } });

  server.begin();
  Serial.println("HTTP server started");
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
    myIp = WiFi.localIP().toString();

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

    // Try to connect to signalk

    if (skServer.length() > 0 && skPort > 0)
    {
      skWsServer->begin(); // Connect to the SignalK TCP server
    }

    // Now start Web Server
    startWebServer();

    currentScreen->draw();
    return true;
  }
  return false;
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{
  state->HandleNMEA2000Msg(N2kMsg, analyze, verbose);
}

void setup_NMEA2000()
{

  NMEA2000.SetProductInformation(&LogProductInformation);
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(LogManufacturerInformation, LogInstallationDescription1, LogInstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(AutopilotSerialNumber,     // Unique number. Use e.g. Serial number.
                                LogDeviceFunction,   // Device function=Autopìlot. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                LogtDeviceClass,      // Device class=Steering and Control Surfaces. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                LogManufacturerCode, // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                LogIndustryGroup     // Industry Group
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

void switchTo(int i)
{
  Screen *oldScreen = currentScreen;

  if (i >= 0 && i < 5)
  {
    if (screens[i] != nullptr)
    {
      if (oldScreen != nullptr)
      {
        oldScreen->exit();
      }
      currentScreen = screens[i];
      currentScreen->enter();
    }
    else
    {
      Serial.print("Screen ");
      Serial.print(i);
      Serial.println(" not implemented");
    }
  }
}

void setup()
{

  M5.begin();
  Serial.begin(115200);
  SD.begin();
  M5.Lcd.setFreeFont(&unicode_24px);
  readPreferences();
  setup_NMEA2000();

  last_touched = millis();

  // M5.Lcd.wakeup();
  Serial.println("Entering Menu Screen");

  // currentScreen = new MenuScreen(TFT_HOR_RES, TFT_VER_RES, "Logs");
  currentScreen = screens[0];
  currentScreen->enter();
}
void loopTask()
{

  if (currentScreen != nullptr)
  {
    int newScreen = currentScreen->run();

    if (newScreen >= 0 && newScreen < 5)
    {
      Serial.println("About to switch");
      switchTo(newScreen);
    }
  }
}

#define GO_SLEEP_TIMEOUT 60000ul // 5 '

void loop()
{
  // NMEA2000.ParseMessages();

  if (!checkConnection())
  {
    startWiFi();
  }
  M5.update();
  server.handleClient();
  skWsServer->run();
  // test_step();

  loopTask();
  delay(1);
}