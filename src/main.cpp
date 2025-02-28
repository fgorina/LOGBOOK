#define TFT_HOR_RES 320
#define TFT_VER_RES 240

#define ESP32_CAN_TX_PIN GPIO_NUM_19
#define ESP32_CAN_RX_PIN GPIO_NUM_27

#include <Arduino.h>
#include <M5Tough.h>
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

static bool redrawData = true;
static bool redrawDistance = true;
static bool recording = false;
static String wifi_ssid = "elrond";          // Store the name of the wireless network.
static String wifi_password = "ailataN1991"; // Store the password of the wireless network.
static IPAddress signalk_tcp_host = IPAddress(192, 168, 1, 2);

M5Display *tft;
ButtonColors on_clrs = {BLUE, CYAN, WHITE};
ButtonColors off_clrs = {BLACK, CYAN, WHITE};
ButtonColors selected_clrs = {RED, WHITE, WHITE};
Button brecord(TFT_HOR_RES / 2 + TFT_HOR_RES / 8, TFT_VER_RES / 2 - 30, TFT_HOR_RES / 4, 60, false, "Start", off_clrs, on_clrs, MC_DATUM);
Button bup(TFT_HOR_RES / 2 + TFT_HOR_RES / 8, 10, TFT_HOR_RES / 4, 60, false, "^", off_clrs, on_clrs, MC_DATUM);
Button bdown(TFT_HOR_RES / 2 + TFT_HOR_RES / 8, TFT_VER_RES - 70, TFT_HOR_RES / 4, 60, false, "v", off_clrs, on_clrs, MC_DATUM);
static long minutes = 0;
static long miles_100 = 0;

long lastTime = millis();

static unsigned long last_touched;

#define DISPLAY_ACTIVE 0
#define DISPLAY_SLEEPING 1
#define DISPLAY_WAKING 2

static int displaySaver = DISPLAY_ACTIVE;
static char buffer[64];

#define MODE_LIST 0
#define MODE_LOG 1

static int mode = MODE_LIST;

#define MAXFILES 100
#define MAXNAME 11 + 5
#define HEADER_SIZE 30
#define FILES_SCREEN 7

char sd_files[MAXFILES][MAXNAME];
int n_files = 0;
int first_file = 0;
int selected_file = -1;

File log_file = File();

// State

tState *state{new tState()};

void updatedDateTime();
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

    updatedDateTime();
    Serial.println(buffer);

    return true;
  }
  return false;
}

// Sorting functions

// Comparison function for qsort

int compareStrings(const void *a, const void *b)
{
  return strcmp((const char *)b, (const char *)a);
}

void sortFiles()
{
  qsort(sd_files, n_files, MAXNAME, compareStrings);
}

// Function to update the date and time

void updatedDateTime()
{

  struct tm timeinfo;
  Serial.println("Getting RTC data");
  if (getLocalTime(&timeinfo))
  {
    strftime(buffer, 64, "RTC %y-%m-%d %H:%M:%S", (const tm *)&timeinfo);
  }
  else
  {
    Serial.println("Failed RTC, getting from time");
    time_t now = time(nullptr);
    struct tm *ti = localtime(&now);
    strftime(buffer, 64, "%y-%m-%d %H:%M:%S", ti);
  }
}

void newFilename()
{
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  strftime(buffer, sizeof(buffer), "/%y%m%d_%H%M.csv", timeinfo);
}

char *duration()
{

  int hours = floor(minutes / 60);
  int mins = minutes - (hours * 60);
  sprintf(buffer, "%d:%d", hours, mins);
  return buffer;
}

char *distance()
{

  int miles = floor(miles_100 / 100);
  int rest = miles_100 - (miles * 100);
  sprintf(buffer, "%d.%d", miles, rest);
  return buffer;
}

void drawData()
{
  if (mode != MODE_LOG)
  {
    return;
  }
  M5.Lcd.setTextSize(1);
  M5.Lcd.fillRect(0, 0, TFT_HOR_RES, 40, BLACK);
  M5.Lcd.setTextDatum(TC_DATUM);
  updatedDateTime();
  M5.Lcd.drawString(buffer, TFT_HOR_RES / 2, 10);
  M5.Lcd.setTextDatum(BL_DATUM);
  redrawData = false;
}

void drawDistance()
{
  if (mode != MODE_LOG)
  {
    return;
  }
  M5.Lcd.setTextSize(1);
  M5.Lcd.fillRect(0, TFT_VER_RES - 80, TFT_HOR_RES, 80, BLACK);
  M5.Lcd.setTextDatum(BL_DATUM);
  M5.Lcd.drawString(duration(), 10, TFT_VER_RES - 10);
  M5.Lcd.setTextDatum(BL_DATUM);
  M5.Lcd.drawString(distance(), TFT_HOR_RES - 10, TFT_VER_RES - 10);
  redrawDistance = false;
}

void drawFile(int file, bool selected)
{
  if (file >= 0 && file < n_files && file >= first_file && file < first_file + FILES_SCREEN)
  {

    M5.Lcd.setTextDatum(CL_DATUM);
    if (selected)
    {
      M5.Lcd.setTextColor(BLACK);
      M5.Lcd.fillRect(0, (file - first_file) * 30 + HEADER_SIZE, TFT_HOR_RES / 2, 30, WHITE);
      M5.Lcd.drawString(sd_files[file], 10, HEADER_SIZE + 15 + (file - first_file) * 30);
    }
    else
    {
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.fillRect(0, (file - first_file) * 30 + HEADER_SIZE, TFT_HOR_RES / 2, 30, BLACK);
      M5.Lcd.drawString(sd_files[file], 10, HEADER_SIZE + 15 + (file - first_file) * 30);
    }
  }
}
void drawSD()
{

  M5.Lcd.fillRect(0, 0, TFT_HOR_RES / 2, TFT_VER_RES, BLACK);
  M5.Lcd.setTextDatum(CC_DATUM);

  M5.Lcd.setTextColor(CYAN);
  M5.Lcd.fillRect(0, 0, TFT_HOR_RES / 2, HEADER_SIZE, BLUE);
  M5.Lcd.drawString("Registres", TFT_HOR_RES / 4, HEADER_SIZE / 2);

  for (int i = first_file; i < n_files && i < first_file + FILES_SCREEN; i++)
  {
    drawFile(i, i == selected_file);
  }
}

void drawScreen()
{

  Serial.println("Drawing Screen");

  // drawData();
  drawSD();
  M5.Buttons.draw();

  // drawDistance();
}

void loadSD()
{
  File root = SD.open("/");
  n_files = 0;
  if (root)
  {

    while (true)
    {
      File entry = root.openNextFile();
      if (!entry)
      {
        // no more files
        break;
      }

      if (entry.name()[0] != '.')
      {
        Serial.println(entry.name());
        strcpy(sd_files[n_files], entry.name());
        n_files++;
        entry.close();
      }
    }
    root.close();
    sortFiles(); // Sort the files after loading them
  }
}
void testSD()
{

  newFilename();
  log_file = SD.open(buffer, FILE_WRITE);
  if (log_file)
  {
    log_file.println("Hello, SD Card!");
    log_file.close();
    Serial.println("Write to SD Card");
    strcpy(sd_files[n_files], buffer);
    n_files++;
    sortFiles(); // Sort the files after loading them
    selected_file = 0;
    first_file = 0;
  }
  else
  {
    Serial.print("Error opening ");
    Serial.println(buffer);
  }
}

void doRecord(Event &e)
{

  recording = !recording;
  brecord.setLabel(recording ? "Stop" : "Start");
  brecord.off = recording ? selected_clrs : off_clrs;

  Serial.println(recording ? "Recording" : "Stopped");

  if (recording)
  {
    testSD();
    minutes = 0;
    miles_100 = 0;
    redrawDistance = true;
    loadSD();
    drawScreen();
  }
  else
  {
    if (log_file)
    {
      log_file.close();
    }
    M5.Buttons.draw();
  }

  // redraw = true;
}

void doScrollUp(Event &e)
{
  first_file = max(first_file - FILES_SCREEN, 0);
  drawSD();
}
void doScrollDown(Event &e)
{
  first_file = min(first_file + FILES_SCREEN, max(n_files - FILES_SCREEN, 0));
  drawSD();
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

void setup()
{

  M5.begin();
  Serial.begin(115200);

  SD.begin();
  loadSD();

  setup_NMEA2000();




  brecord.addHandler(doRecord, E_PRESSED);
  bup.addHandler(doScrollUp, E_PRESSED);
  bdown.addHandler(doScrollDown, E_PRESSED);



  last_touched = millis();

  M5.Lcd.wakeup();
  drawScreen();
}

void loopTask()
{

  if (recording)
  {
    minutes += 1;
    miles_100 += 1;
    redrawDistance = true;
  }
}

#define GO_SLEEP_TIMEOUT 60000ul // 5 '

void do_select_file()
{
  last_touched = millis();

  // Now compute the line
  if (M5.Touch.point[0].x >= 0 && M5.Touch.point[0].y >= HEADER_SIZE)
  {
    int line = floor((M5.Touch.point[0].y - HEADER_SIZE) / 30) + first_file;
    if (line < n_files)
    {
      Serial.print("Clicked on file ");
      Serial.print(line);
      Serial.print(" : ");
      Serial.println(sd_files[line]);
      drawFile(selected_file, false);
      selected_file = line;
      drawFile(selected_file, true);
    }
    else
    {
      Serial.print("Clicked outside file ");
      Serial.println(line);
      drawFile(selected_file, false);
      selected_file = -1;
    }
  }
}

void do_touch()
{
  if (M5.Touch.point[0].x > TFT_HOR_RES / 2 && M5.Touch.point[0].x < TFT_HOR_RES)
  {
  }
  else
  {
    do_select_file();
  }
}

void loop()
{
  NMEA2000.ParseMessages();
  if (!checkConnection())
  {
    startWiFi();
  }
  M5.update();

  long m = millis();
  if (m - lastTime > 1000 && mode == MODE_LOG)
  {
    drawData();
    lastTime = m;
    loopTask();
  }
  else
  {
    if (M5.Touch.changed)
    {
      do_touch();
    }
  }

  if (last_touched > 0 && (millis() - last_touched > GO_SLEEP_TIMEOUT) && false)
  {
    // disconnect_clients();
    // save_page(page);
    // deep_sleep_with_touch_wakeup();
    Serial.println("Going to sleep");
    last_touched = 0;
    displaySaver = DISPLAY_SLEEPING;
    M5.Lcd.sleep(); // powerSaveOn();
  }
  else if (last_touched != 0 && displaySaver == DISPLAY_WAKING)
  {
    Serial.println("Waking Up");
    displaySaver = DISPLAY_ACTIVE;
    M5.Lcd.wakeup(); // powerSaveOff();
    redrawData = true;
    redrawDistance = true;
  }

  if (redrawData)
  {
    drawData();
  }
  if (redrawDistance)
  {
    drawDistance();
  }
  delay(5);
}