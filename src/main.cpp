#define TFT_HOR_RES 320
#define TFT_VER_RES 240

#define ESP32_CAN_TX_PIN GPIO_NUM_19
#define ESP32_CAN_RX_PIN GPIO_NUM_27

#include <Arduino.h>
#include <WebServer.h>
#include <M5Unified.h>
#include <time.h>
#include <Preferences.h>
#include "esp_task_wdt.h"
#include <NMEA2000_esp32.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include "N2kDeviceList.h"
#include <ArduinoWebsockets.h>
#include "net_signalk.h"
#include "net_nmea0183.h"

#include "PyTypes.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>

#include "State.h"

#include <SD.h>

#include <FS.h>
#include <SPIFFS.h>

#ifndef FORMAT_SPIFFS_IF_FAILED
#define FORMAT_SPIFFS_IF_FAILED true
#endif

#include "Utils.h"
// Screens

#include "MenuScreen.h"
#include "SDScreen.h"
#include "RecordScreen.h"
#include "WaitScreen.h"
#include "InfoScreen.h"
#include "N2KDevices.h"

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

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char LogManufacturerInformation[] PROGMEM = "Paco Gorina, fgorina@gmail.com";
const char LogInstallationDescription1[] PROGMEM = "Just connect and configure with a web browser";
const char LogInstallationDescription2[] PROGMEM = "Select NMEA 2000, SignalK and WiFi and format settings";

const unsigned long AutopilotSerialNumber PROGMEM = 13;
const unsigned char LogDeviceFunction PROGMEM = 140; // Log Recorder
const unsigned char LogtDeviceClass = 20;            // Safety Systems
const uint16_t LogManufacturerCode = 2046;           // Free?
const unsigned char LogIndustryGroup = 4;            // Marine

// Global variables + State

#ifdef DEV
static String wifi_ssid = "elrond";          //"TP-LINK_2695";//"Yamato"; //"starlink_mini";   // Store the name of the wireless network.
static String wifi_password = "ailataN1991"; // "39338518"; //ailataN1991"; // Store the password of the wireless network.
static String skServer = "192.168.001.150";  //"192.168.1.54";
int skPort = 3000;
bool useN2k = false;
bool useSK = true;
bool use0183 = false;
#else
static String wifi_ssid = "Yamato";          //"TP-LINK_2695";//"Yamato"; //"starlink_mini";   // Store the name of the wireless network.
static String wifi_password = "ailataN1991"; // "39338518"; //ailataN1991"; // Store the password of the wireless network.
static String skServer = "192.168.1.2";      //"192.168.1.54";
int skPort = 3000;
bool useN2k = true;
bool useSK = false;
bool use0183 = false;
#endif

static String n2kSources = "15";
int sources[MAX_SOURCES] = {15, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int n_sources = 1;

// static IPAddress signalk_tcp_host = IPAddress(192,168,1,204); //IPAddress(192, 168, 1, 2);

bool starting = true; // Is true if WiFi is not configured
// Http Server
WebServer server(80);

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

// Test variables

double speed = 5.0;   // 5 knots
double heading = 0.0; // In radians

// State

tState *state{new tState()};

String myIp = "Connecting...";

// SignalK server

NetSignalkWS *skWsServer = new NetSignalkWS(skServer.c_str(), skPort, state);
NetNMEA0183  *nmea0183   = new NetNMEA0183(skServer.c_str(), 10110, state);

// Screens

Screen *screens[6] = {
    new MenuScreen(state, TFT_HOR_RES, TFT_VER_RES, "Logs"),
    new RecordScreen(TFT_HOR_RES, TFT_VER_RES, "Record", state, 1000),
    new SDScreen(TFT_HOR_RES, TFT_VER_RES, "Logs", state),
    new InfoScreen(wifi_ssid, &myIp, &useN2k, &useSK, &use0183, &skServer, &skPort, &n2kSources, TFT_HOR_RES,  TFT_VER_RES, "Info"),
    new N2KDevices(pN2kDeviceList, TFT_HOR_RES,  TFT_VER_RES, "N2k"),
    nullptr,
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

  useN2k  = preferences.getBool("USEN2K",  false);
  useSK   = preferences.getBool("USESK",   false);
  use0183 = preferences.getBool("USE0183", false);
  

  n2kSources = preferences.getString("N2KSOURCES", n2kSources);
  n_sources = splitter((char*) (n2kSources.c_str()), sources, ',', n2kSources.length(), MAX_SOURCES);
  for(int i = n_sources; i < MAX_SOURCES; i++){
    sources[i] = -1;
  }

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
  Serial.print("use N2k : ");
  Serial.println(useN2k);
  Serial.print("use SignalK : ");
  Serial.println(useSK);
  Serial.print("use NMEA0183: ");
  Serial.println(use0183);
  Serial.print("Sources : ");
  for(int i = 0; i < MAX_SOURCES; i++){
    if (sources[i] >= 0){
      Serial.print(sources[i]);
      Serial.print(",");
    }
  }
  Serial.println();
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

String getFullUri(String last)
{
  return "http://logbook.local/" + last;
}

void handleHelp()
{
  Serial.println("handleHelp");
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
  {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  File file = SPIFFS.open("/help.html", "r");
  if (!file)
  {
    Serial.println("File not found");
  }
  else
  {
    server.streamFile(file, "text/html");
    file.close();
  }
  SPIFFS.end();
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

  String output = "<htlm><head><title>Logs</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0</head><body>\n";

  output += "<h1><a href=\"" + getFullUri("index.html") + "\">Logbook</a>/Logs</h1>\n";
  output += "<a href=\"" + getFullUri("ask") + "\">Esborrar tots els Logs</a></br>";
  output += "<ul>\n";
  if (root.isDirectory())
  {
    File file = root.openNextFile();
    while (file)
    {
      if (file.name()[0] != '.')
      {
        String path = file.path();
        output += String("<li><a href=\"" + getFullUri(file.path()) + "\">") + file.name() + "</a>    <a href=\"" + getFullUri(String("del/" + path)) + "\">Delete</a></li>\n";
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
  Serial.println("handleMenu");
  String output = "<html><head>"
                  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
                  "<title>Logbook by Paco Gorina</title>"
                  "</head><body>";

  output += "<h1>Logbook by Paco Gorina</h1>";
  output += "<ul>";
  output += "<li><a href=\"" + getFullUri("prefs") + "\">Prefer&egrave;ncies</a></li>";
  output += "<li><a href=\"" + getFullUri("logs") + "\">Logs</a></li>";
  output += "<li><a href=\"" + getFullUri("restart") + "\">Restart</a></li>";
  output += "</ul>";
  output += "</body></html>";
  unsigned long len = output.length();
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);
}

void handleAskForDelete()
{
  Serial.println("handleAskForDelete");
  String output = "<html><head><title>Confirmeu, si us plau</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0</head><body>";
  output += "Segur que voleu esborrar tots els logs? <a href=" + getFullUri("clear") + ">Si</a> <a href=" + getFullUri("logs") + ">No</a>";
  unsigned long len = output.length();
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);
}
void handleDeleteAll()
{
  Serial.println("handleDeleteAll");

  File root = SD.open("/");

  String output = "<htlm><head><meta http-equiv=\"refresh\" content=\"0;url=/\"><title>Logs</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0</head><body>\n";
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
  n2kSources = join(sources, MAX_SOURCES, ',');
  String output = "<html><head><title>Prefer&egrave;ncies</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0</head><body>";
  output += "<h1><a href=\"" + getFullUri("index.html") + "\">Logbook</a>/Prefer&egrave;ncies</h1>";
  output += "<form action=\"" + getFullUri("updatePrefs") + "\" method=\"post\">";
  output += "<table border=0>";
  output += "<tr><td><label for=\"ssid\">SSID:</label></td><td><input type=\"text\" id=\"ssid\" name=\"ssid\" value=\"" + wifi_ssid + "\"></td></tr>";
  output += "<tr><td><label for=\"password\">Password:</label></td><td><input type=\"password\" id=\"password\" name=\"password\" value=\"" + wifi_password + "\"></td></tr>";
  output += "<tr><td><label for=\"skserver\">SignalK Server:</label></td><td><input type=\"text\" id=\"skserver\" name=\"skserver\" value=\"" + skServer + "\"></td></tr>";
  output += "<tr><td><label for=\"skport\">SignalK Port:</label></td><td><input type=\"number\" id=\"skport\" name=\"skport\" value=\"" + String(skPort) + "\"></td></tr>";
  output += "<tr><td><label for=\"usexml\">Use GPX:</label></td><td><input type=\"checkbox\" id=\"usexml\" name=\"usexml\" value=\"on\" " + String(((RecordScreen *)screens[1])->xmlFormat ? "checked" : "") + "></td></tr>";
  output += "<tr><td><label for=\"usen2k\">Use N2k:</label></td><td><input type=\"checkbox\" id=\"usen2k\" name=\"usen2k\" value=\"on\" " + String(useN2k ? "checked" : "") + "></td></tr>";
  output += "<tr><td><label for=\"n2kdevices\">N2K Devices:</label></td><td><input type=\"text\" id=\"n2kdevices\" name=\"n2kdevices\" value=\"" + n2kSources + "\"></td></tr>";
  output += "<tr><td><label for=\"usesk\">Use SignalK:</label></td><td><input type=\"checkbox\" id=\"usesk\" name=\"usesk\" value=\"on\" " + String(useSK ? "checked" : "") + "></td></tr>";
  output += "<tr><td><label for=\"use0183\">Use NMEA 0183:</label></td><td><input type=\"checkbox\" id=\"use0183\" name=\"use0183\" value=\"on\" " + String(use0183 ? "checked" : "") + "></td></tr>";
  output += "<tr><td colspan=2 align=center><input type=\"submit\" value=\"Submit\"></td></tr>";
  output += "</table>";
  output += "</form>";
  output += "</body></html>";
  unsigned long len = output.length();
  server.sendHeader("Cache-Control", "no-cache");
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
    ((RecordScreen *)screens[1])->xmlFormat = true;
  }
  else
  {
    ((RecordScreen *)screens[1])->xmlFormat = false;
  }

  if (server.hasArg("usen2k"))
  {
    useN2k = true;
  }
  else
  {
    useN2k = false;
  }
  if (server.hasArg("usesk"))
  {
    useSK = true;
  }
  else
  {
    useSK = false;
  }
  use0183 = server.hasArg("use0183");
  if (server.hasArg("n2kdevices")){
    n2kSources = server.arg("n2kdevices");
    n_sources = splitter((char*) (n2kSources.c_str()), sources, ',', n2kSources.length(), MAX_SOURCES);
    for(int i = n_sources; i < MAX_SOURCES; i++){
      sources[i] = -1;
    }
  }
  writePreferences();
  server.sendHeader("Location", getFullUri("index.html"), true);
  server.send(302, "text/plain", "");
}

void handleRestart()
{
  server.sendHeader("Location", getFullUri("index.html"), true);
  server.send(302, "text/plain", "");
  Serial.println("Restarting");
  ESP.restart();
}
// WiFI
boolean checkConnection()
{                // Check wifi connection.
  int count = 0; // count.
  while (count < 1000)
  { // If you fail to connect to wifi within 30*350ms (10.5s), return false; otherwise return true.
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }
    delay(10);
    count++;
  }
  return false;
}

void startWebServer()
{
  server.on("/", HTTP_GET, handleMenu);
  server.on("/index.html", HTTP_GET, handleMenu);
  server.on("/ask", HTTP_GET, handleAskForDelete);
  server.on("/logs", HTTP_GET, handleFileList);
  server.on("/prefs", HTTP_GET, handlePreferences);
  server.on("/updatePrefs", HTTP_POST, handleUpdatePreferences);
  server.on("/clear", HTTP_GET, handleDeleteAll);
  server.on("/help", HTTP_GET, handleHelp);
  server.on("/restart", HTTP_GET, handleRestart);

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
boolean startWiFiAP()
{
  Serial.println("Creating wifi logbook / 123456 ");
  WiFi.mode(wifi_mode_t::WIFI_MODE_AP);
  WiFi.softAP("logbook", "12345678");
  IPAddress IP = WiFi.softAPIP();
  Serial.println("Ip : " + IP.toString());

  // Start mdns so we have a name

  if (!MDNS.begin("logbook"))
  {
    Serial.println("Error setting up MDNS responder!");
  }
  else
  {
    Serial.println("mDNS responder started");
  }

  startWebServer();
  starting = false;
  return true;
}
boolean startWiFi()
{ // Check whether there is wifi configuration information storage, if there is return 1, if no return 0.

  // WiFi.setAutoConnect(true);

  Serial.println("Connecting to ");
  Serial.print(wifi_ssid);
  Serial.print(" ");
  Serial.println(wifi_password);
  WiFi.mode(wifi_mode_t::WIFI_MODE_STA);
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
    vTaskDelay(5);
    // Start mdns so we have a name

    if (!MDNS.begin("logbook"))
    {
      Serial.println("Error setting up MDNS responder!");
    }
    else
    {
      Serial.println("mDNS responder started");
    }
    // Try to connect to signalk
    vTaskDelay(5);
    if (skServer.length() > 0 && skPort > 0 && useSK)
    {
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
bool checkSource(unsigned char source){
  if(n_sources == 0){
    return true;
  }

  for(int i = 0; i < n_sources; i++){
    if(sources[i] == source){
      return true;
    }
  }
  return false;
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{
  if (checkSource(N2kMsg.Source))
  {
    state->HandleNMEA2000Msg(N2kMsg, analyze, verbose);
  }
}

void setup_NMEA2000()
{

  NMEA2000.SetProductInformation(&LogProductInformation);
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(LogManufacturerInformation, LogInstallationDescription1, LogInstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(AutopilotSerialNumber, // Unique number. Use e.g. Serial number.
                                LogDeviceFunction,     // Device function=Autopìlot. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                LogtDeviceClass,       // Device class=Steering and Control Surfaces. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                LogManufacturerCode,   // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                LogIndustryGroup       // Industry Group
  );

  NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on theTransmitTransmit  bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 25);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);  ttttrtt   // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(true); // Disable all msg forwarding to USB (=Serial)

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
  screens[4] = new N2KDevices(pN2kDeviceList, TFT_HOR_RES,  TFT_VER_RES, "Devices");
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
/// Tasks

TaskHandle_t taskNetwork;
TaskHandle_t taskN2K;
TaskHandle_t taskWss;
TaskHandle_t task0183;

void networkTask(void *parameter)
{

  while (true)
  {
    if (!checkConnection() && !wifi_ssid.isEmpty())
    {
      startWiFi();
    }

    server.handleClient();
   
    vTaskDelay(10);
  }
}

void n2KTask(void *parameter)
{

  while (true)
  {
   
    if (useN2k)
    {
      NMEA2000.ParseMessages();
    }

    vTaskDelay(10);
  }
}

void wssTask(void *parameter)
{
  while (true)
  {
    if (useSK && checkConnection())
    {
      skWsServer->run();
    }
    vTaskDelay(10);
  }
}

void nmea0183Task(void *parameter)
{
  while (true)
  {
    if (use0183 && checkConnection())
    {
      nmea0183->run();
    }
    vTaskDelay(20);
  }
}
void uiTask(const m5::touch_detail_t &t)
{

    if (currentScreen != nullptr)
    {
      int newScreen = currentScreen->run(t);

      if (newScreen >= 0 && newScreen < 5)
      {
        Serial.println("About to switch");
        switchTo(newScreen);
      }
    }

}



void resetNetwork()
{
  // Sets preferences to work as STA
  // with ssig "logbook"
  // and passwd "123456"
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

void splash()
{
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
  M5.Display.drawString("Use Nemea 2000: " + String(useN2k ? "Si" : "No"), 10, 130);
  M5.Display.drawString("Use SignalK: " + String(useSK ? "Si" : "No"), 10, 170);

  M5.Display.setTextDatum(CC_DATUM);
  M5.Display.drawString("Toqueu per reset", TFT_HOR_RES / 2, 200);

  while (millis() - s < duration)
  {
    M5.update();
    auto count = M5.Touch.getCount();
    if (count > 0)
    {
      auto t = M5.Touch.getDetail(0);
      if (t.wasPressed())
      {
        press = millis();

      }
      else if (t.wasReleased())
      {
        if (millis() - press > 1000)
        {
          Serial.println("Resetting Network");
          resetNetwork();
        }
        else
        {
          press = -1;
        }
      }
    }
    delay(1);
  }
  return;
}


void setup()
{

  M5.begin();
  M5.Display.setRotation(1);
  Serial.begin(115200);
  M5.Display.wakeup();
  readPreferences();
  if (!wifi_ssid.isEmpty())
  {
    splash();
  }
  else
  {
    startWiFiAP();
  }

  M5.Display.setFont(&fonts::FreeSans12pt7b);
  M5.Display.setTextSize(1.0);
  // Core2: SD uses VSPI, CS=GPIO4, CLK=GPIO18, MISO=GPIO38, MOSI=GPIO23
  SPI.begin(18, 38, 23, 4);
  if (!SD.begin(4, SPI, 25000000)) {
    Serial.println("SD card mount failed");
  } else {
    Serial.println("SD card mounted");
  }

  if (useN2k)
  {
    setup_NMEA2000();
  }
  

  // M5.Lcd.wakeup();
  Serial.println("Starting Tasks");
  xTaskCreate(networkTask, "NetworkTask",4000,NULL,1,&taskNetwork);
  Serial.println("Network Task Created");
  if(useN2k){ 
     xTaskCreate(n2KTask, "N2kTask",4000,NULL,0,&taskN2K);
     Serial.println("N2K Task Created");
  }

  if(useSK){
    xTaskCreate(wssTask, "WSS Task",4000,NULL,0,&taskWss);
    Serial.println("WSS Task Created");
  }

  if(use0183){
    xTaskCreate(nmea0183Task, "NMEA0183 Task",4000,NULL,0,&task0183);
    Serial.println("NMEA0183 Task Created");
  }
 
  // currentScreen = new MenuScreen(TFT_HOR_RES, TFT_VER_RES, "Logs");

  Serial.println("Opening main screen");
  if (wifi_ssid.isEmpty())
  {
    currentScreen = screens[3];
  }
  else
  {
    currentScreen = screens[0];
  }
  currentScreen->enter();
  last_touched = millis();

}

#define GO_SLEEP_TIMEOUT 30000ul // 5 '

void* p;

void loop()
{
  M5.update();
  auto &t = M5.Touch.getDetail(0);
  uiTask(t);
  
    auto count = M5.Touch.getCount();
    if (count > 0)
    {
      
      if (t.wasPressed() || t.wasReleased())
      {
        last_touched = millis();
        Serial.println("Touched");
        if (state->displaySaver == DISPLAY_SLEEPING && t.wasPressed())
        {
          Serial.println("Waking Up");
          M5.Display.wakeup();
          M5.Display.setBrightness(128);
          state->displaySaver = DISPLAY_WAKING;
        }
        else if (state->displaySaver == DISPLAY_WAKING && t.wasReleased())
        {
          Serial.println("Activating");
          state->displaySaver = DISPLAY_ACTIVE;
          last_touched = millis();
        }
      }
    }
  
  if (millis() - last_touched  > GO_SLEEP_TIMEOUT && state->displaySaver == DISPLAY_ACTIVE )
  {
    Serial.println("Going to Sleep");
    M5.Display.sleep();
    M5.Display.setBrightness(0);
    state->displaySaver = DISPLAY_SLEEPING;
  }

  vTaskDelay(50);
}