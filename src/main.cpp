#define TFT_HOR_RES 320
#define TFT_VER_RES 240

#include <Arduino.h>
#include <M5Tough.h>

#include <time.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <SD.h>

// Global variables + State

static bool redrawData = true;
static bool redrawDistance = true;
static bool recording = false;
static String wifi_ssid = "elrond";          // Store the name of the wireless network.
static String wifi_password = "ailataN1991"; // Store the password of the wireless network.
static IPAddress signalk_tcp_host = IPAddress(192, 168, 1, 2);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org");


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
    if (!getLocalTime(&timeinfo) || timeinfo.tm_year < 20){
      configTime(0, 0, "europe.pool.ntp.org");
      Serial.println("Synced RTC from NTP");
    }else{
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


int compareStrings(const void *a, const void *b) {
  return strcmp((const char *)b, (const char *)a);
}

void sortFiles() {
  qsort(sd_files, n_files, MAXNAME, compareStrings);
}

// Function to update the date and time

void updatedDateTime()
{

  struct tm timeinfo;
  Serial.println("Getting RTC data");
  if (getLocalTime(&timeinfo)){
    strftime(buffer, 64, "RTC %y-%m-%d %H:%M:%S", (const tm*)&timeinfo);

  }else{
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
    selected_file =0;
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
    if(log_file){
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
void setup()
{

  M5.begin();
  Serial.begin(115200);
  Serial.println("Initializing lvgl");
  Serial.println("lvgl initialized");

  brecord.addHandler(doRecord, E_PRESSED);
  bup.addHandler(doScrollUp, E_PRESSED);
  bdown.addHandler(doScrollDown, E_PRESSED);

  SD.begin();
  loadSD();

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