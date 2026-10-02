/* Configuration web server

    Pages served on http://<deviceName>.local/ or http://<ip>/ : menu, preferences, logs
    (list, download, delete), help, restart and firmware update.

    Uses ESPAsyncWebServer: handlers run in the AsyncTCP task and several
    connections are served at once, so a browser's idle extra connections
    don't stall the page (the synchronous WebServer served one at a time).
*/

#include "LogWebServer.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <SD.h>
#include <SPIFFS.h>
#include <Update.h>
#include <memory>

#include "BuildInfo.h"
#include "Constants.h"
#include "RecordScreen.h"
#include "State.h"
#include "Utils.h"

#ifndef FORMAT_SPIFFS_IF_FAILED
#define FORMAT_SPIFFS_IF_FAILED true
#endif

// Globals owned by main.cpp

extern String wifi_ssid;
extern String wifi_password;
extern String skServer;
extern int skPort;
extern bool useN2k;
extern bool useSK;
extern bool use0183;
extern String n2kSources;
extern int sources[MAX_SOURCES];
extern int n_sources;
extern String deviceName;
extern tState *state;
extern Screen *screens[6];
extern SemaphoreHandle_t sdMutex;

void writePreferences();
void writeSailState();

static AsyncWebServer server(80);

// Handlers can't restart the device themselves: the response would never be
// sent. They set this and the network task restarts once it is due.
static volatile unsigned long restartAt = 0;

static void scheduleRestart() {
  restartAt = millis() + 1000;
  if (restartAt == 0) {
    restartAt = 1;
  }
}

void handleWebServer() {
  if (restartAt != 0 && (long)(millis() - restartAt) >= 0) {
    Serial.println("Restarting");
    ESP.restart();
  }
}

// The recorder and the uploader also use the SD, from other tasks. Don't
// block the AsyncTCP task waiting for them: answer 503 and let the browser retry.
static const TickType_t kLockTimeout = pdMS_TO_TICKS(200);

static bool tryLockSD() { return xSemaphoreTake(sdMutex, kLockTimeout) == pdTRUE; }

static void unlockSD() { xSemaphoreGive(sdMutex); }

static void sendBusy(AsyncWebServerRequest *request) {
  AsyncWebServerResponse *r = request->beginResponse(503, "text/plain", "SD busy");
  r->addHeader("Retry-After", "1");
  request->send(r);
}


String getContentType(AsyncWebServerRequest *request, String filename) {
  if (request->hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  } else if (filename.endsWith(".gpx")) {
    return "application/gpx+xml";
  } else if (filename.endsWith(".csv")) {
    return "text/plain";
  }
  return "text/plain";
}

// Root-relative, so links keep whatever host the browser used: the mDNS name
// locally or the IP over a VPN. Paths already starting with "/" (log files)
// are returned as is, as "//logs/..." would be read as a host name.
String getFullUri(String last) {
  if (last.startsWith("/")) {
    return last;
  }
  return "/" + last;
}

void handleHelp(AsyncWebServerRequest *request) {
  Serial.println("handleHelp");
  // Stays mounted: the file is read after this handler returns
  static bool spiffsMounted = false;
  if (!spiffsMounted) {
    spiffsMounted = SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED);
  }
  if (!spiffsMounted) {
    Serial.println("SPIFFS Mount Failed");
    request->send(500, "text/plain", "SPIFFS Mount Failed");
    return;
  }
  if (!SPIFFS.exists("/help.html")) {
    Serial.println("File not found");
    request->send(404, "text/plain", "FileNotFound");
    return;
  }
  request->send(SPIFFS, "/help.html", "text/html");
}

void handleFileRead(AsyncWebServerRequest *request) {
  String path = request->url();
  Serial.printf("Downloading %s\n", path.c_str());
  if (path.endsWith("/")) {
    path += "index.htm";
  }

  if (!tryLockSD()) {
    sendBusy(request);
    return;
  }
  File file = SD.exists(path) ? SD.open(path, FILE_READ) : File();
  bool isFile = file && !file.isDirectory();
  if (file && !isFile) {
    file.close();
  }
  unlockSD();

  if (!isFile) {
    Serial.println("File " + path + " not found.");
    request->send(404, "text/plain", "FileNotFound");
    return;
  }

  Serial.println("handleFileRead: " + path);
  // Chunked so every SD read happens under sdMutex (the recorder and the
  // uploader share the card). The shared_ptr closes the file when the
  // response is destroyed.
  auto f = std::make_shared<File>(file);
  String name = path.substring(path.lastIndexOf('/') + 1);
  AsyncWebServerResponse *r = request->beginChunkedResponse(
      getContentType(request, path),
      [f](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        if (!tryLockSD()) {
          return RESPONSE_TRY_AGAIN;
        }
        size_t n = f->read(buffer, maxLen);
        unlockSD();
        return n;  // 0 ends the response
      });
  if (request->hasArg("download") || path.startsWith("/logs/")) {
    r->addHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
  }
  request->send(r);
}

// Directory listing in progress, kept alive by the chunked response
struct LogListState {
  File root;
  String pending;
  bool done = false;
  ~LogListState() {
    if (root) {
      root.close();
    }
  }
};

void handleFileList(AsyncWebServerRequest *request) {
  // "/logs" also matches "/logs/<file>": those are downloads
  if (request->url() != "/logs") {
    handleFileRead(request);
    return;
  }
  Serial.println("handleFileList");

  auto st = std::make_shared<LogListState>();
  if (!tryLockSD()) {
    sendBusy(request);
    return;
  }
  st->root = SD.open("/logs");
  unlockSD();

  st->pending = "<html><head><title>Logs</title>"
                "<meta name=\"viewport\" content=\"width=device-width, "
                "initial-scale=1.0\">"
                "</head><body>\n";
  st->pending += "<h1><a href=\"" + getFullUri("index.html") + "\">" +
                 deviceName + "</a>/Logs</h1>\n";
  st->pending += "<a href=\"" + getFullUri("ask") +
                 "\">Esborrar tots els Logs</a><br>\n";
  st->pending += "<ul>\n";

  // Send in chunks: avoids building a large String in heap and lets the
  // browser receive data progressively without waiting for the full page.
  request->sendChunked(
      "text/html",
      [st](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        if (!st->done && st->pending.length() < maxLen && tryLockSD()) {
          while (!st->done && st->pending.length() < maxLen) {
            File file = (st->root && st->root.isDirectory())
                            ? st->root.openNextFile()
                            : File();
            if (!file) {
              if (st->root) {
                st->root.close();
              }
              st->pending += "</ul></body></html>\n";
              st->done = true;
            } else {
              if (file.name()[0] != '.') {
                String path = file.path();
                st->pending += "<li><a href=\"" + getFullUri(path) + "\">" +
                               file.name() +
                               "</a>&nbsp;&nbsp;"
                               "<a href=\"" +
                               getFullUri("del/" + path) + "\">Delete</a></li>\n";
              }
              file.close();
            }
          }
          unlockSD();
        }
        if (st->pending.isEmpty()) {
          return st->done ? 0 : RESPONSE_TRY_AGAIN;
        }
        size_t n = min(maxLen, (size_t)st->pending.length());
        memcpy(buffer, st->pending.c_str(), n);
        st->pending.remove(0, n);
        return n;
      });
}

void handleMenu(AsyncWebServerRequest *request) {
  Serial.println("handleMenu");
  String output = "<html><head>"
                  "<meta name=\"viewport\" content=\"width=device-width, "
                  "initial-scale=1.0\">"
                  "<title>" +
                  deviceName +
                  " by Paco Gorina</title>"
                  "</head><body>";

  output += "<h1>Logbook by Paco Gorina</h1>";
  output += "<ul>";
  output += "<li><a href=\"" + getFullUri("prefs") +
            "\">Prefer&egrave;ncies</a></li>";
  output += "<li><a href=\"" + getFullUri("logs") + "\">Logs</a></li>";
  output += "<li><a href=\"" + getFullUri("restart") + "\">Restart</a></li><hr>";
  output += "<li><a href=\"" + getFullUri("update") +
            "\">Firmware Update</a></li>";

  output += "</ul>";
  output += "<p><small>Firmware: " FW_VERSION " (" FW_BUILD_TIME ")</small></p>";
  output += "</body></html>";
  request->send(200, "text/html", output);
}

void handleAskForDelete(AsyncWebServerRequest *request) {
  Serial.println("handleAskForDelete");
  String output =
      "<html><head><title>Confirmeu, si us plau</title><meta name=\"viewport\" "
      "content=\"width=device-width, initial-scale=1.0</head><body>";
  output +=
      "Segur que voleu esborrar tots els logs? <a href=" + getFullUri("clear") +
      ">Si</a> <a href=" + getFullUri("logs") + ">No</a>";
  request->send(200, "text/html", output);
}
void handleDeleteAll(AsyncWebServerRequest *request) {
  Serial.println("handleDeleteAll");

  if (!tryLockSD()) {
    sendBusy(request);
    return;
  }
  File root = SD.open("/");

  String output =
      "<htlm><head><meta http-equiv=\"refresh\" "
      "content=\"0;url=/\"><title>Logs</title><meta name=\"viewport\" "
      "content=\"width=device-width, initial-scale=1.0</head><body>\n";
  output += "<h1>Logs</h1>\n";
  output += "<ul>\n";
  if (root.isDirectory()) {
    File file = root.openNextFile();
    while (file) {
      if (file.name()[0] != '.') {
        SD.remove(file.path());
      }

      file = root.openNextFile();
    }
  }
  root.close();
  unlockSD();

  output += "</ul>\n";
  request->send(200, "text/html", output);
}

void deleteFile(AsyncWebServerRequest *request) {
  String uri = request->url();
  Serial.println("deleteFile uri " + uri);
  String path = uri.substring(4, uri.length());
  Serial.println("deleteFile " + path);
  if (!tryLockSD()) {
    sendBusy(request);
    return;
  }
  SD.remove(path);
  unlockSD();
  // Back to the list by redirect, so reloading it doesn't delete again
  request->redirect(getFullUri("logs"));
}

void handlePreferences(AsyncWebServerRequest *request) {
  Serial.println("handlePreferences");
  n2kSources = join(sources, MAX_SOURCES, ',');
  String output =
      "<html><head><title>Prefer&egrave;ncies</title><meta name=\"viewport\" "
      "content=\"width=device-width, initial-scale=1.0</head><body>";
  output += "<h1><a href=\"" + getFullUri("index.html") + "\">" + deviceName +
            "</a>/Prefer&egrave;ncies</h1>";
  output +=
      "<form action=\"" + getFullUri("updatePrefs") + "\" method=\"post\">";
  output += "<table border=0>";
  output += "<tr><td><label for=\"ssid\">SSID:</label></td><td><input "
            "type=\"text\" id=\"ssid\" name=\"ssid\" value=\"" +
            wifi_ssid + "\"></td></tr>";
  output += "<tr><td><label for=\"password\">Password:</label></td><td><input "
            "type=\"password\" id=\"password\" name=\"password\" value=\"" +
            wifi_password + "\"></td></tr>";
  output +=
      "<tr><td><label for=\"skserver\">SignalK Server:</label></td><td><input "
      "type=\"text\" id=\"skserver\" name=\"skserver\" value=\"" +
      skServer + "\"></td></tr>";
  output +=
      "<tr><td><label for=\"skport\">SignalK Port:</label></td><td><input "
      "type=\"number\" id=\"skport\" name=\"skport\" value=\"" +
      String(skPort) + "\"></td></tr>";
  output += "<tr><td><label for=\"usexml\">Use GPX:</label></td><td><input "
            "type=\"checkbox\" id=\"usexml\" name=\"usexml\" value=\"on\" " +
            String(((RecordScreen *)screens[1])->xmlFormat ? "checked" : "") +
            "></td></tr>";
  output += "<tr><td><label for=\"usen2k\">Use N2k:</label></td><td><input "
            "type=\"checkbox\" id=\"usen2k\" name=\"usen2k\" value=\"on\" " +
            String(useN2k ? "checked" : "") + "></td></tr>";
  output +=
      "<tr><td><label for=\"n2kdevices\">N2K Devices:</label></td><td><input "
      "type=\"text\" id=\"n2kdevices\" name=\"n2kdevices\" value=\"" +
      n2kSources + "\"></td></tr>";
  output += "<tr><td><label for=\"usesk\">Use SignalK:</label></td><td><input "
            "type=\"checkbox\" id=\"usesk\" name=\"usesk\" value=\"on\" " +
            String(useSK ? "checked" : "") + "></td></tr>";
  output +=
      "<tr><td><label for=\"use0183\">Use NMEA 0183:</label></td><td><input "
      "type=\"checkbox\" id=\"use0183\" name=\"use0183\" value=\"on\" " +
      String(use0183 ? "checked" : "") + "></td></tr>";
  output += "<tr><td colspan=2><h3>Sails on board</h3></td></tr>";
  for (int i = 0; i < tState::N_SAILS; i++) {
    String id = "sail" + String(i);
    output += "<tr><td><label for=\"" + id + "\">" + tState::SAIL_NAMES[i] +
              ":</label></td><td><input type=\"checkbox\" id=\"" + id +
              "\" name=\"" + id + "\" value=\"on\" " +
              String(state->hasSail(i) ? "checked" : "") + "></td></tr>";
  }
  output += "<tr><td colspan=2 align=center><input type=\"submit\" "
            "value=\"Submit\"></td></tr>";
  output += "</table>";
  output += "</form>";
  output += "</body></html>";
  AsyncWebServerResponse *response =
      request->beginResponse(200, "text/html", output);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
}

void handleUpdatePreferences(AsyncWebServerRequest *request) {

  Serial.println("handleUpdatePreferences");
  if (request->hasArg("ssid")) {
    wifi_ssid = request->arg("ssid");
  }
  if (request->hasArg("password")) {
    wifi_password = request->arg("password");
  }
  if (request->hasArg("skserver")) {
    skServer = request->arg("skserver");
  }
  if (request->hasArg("skport")) {
    skPort = request->arg("skport").toInt();
  }
  if (request->hasArg("usexml")) {
    ((RecordScreen *)screens[1])->xmlFormat = true;
  } else {
    ((RecordScreen *)screens[1])->xmlFormat = false;
  }

  if (request->hasArg("usen2k")) {
    useN2k = true;
  } else {
    useN2k = false;
  }
  if (request->hasArg("usesk")) {
    useSK = true;
  } else {
    useSK = false;
  }
  use0183 = request->hasArg("use0183");
  if (request->hasArg("n2kdevices")) {
    n2kSources = request->arg("n2kdevices");
    n_sources = splitter((char *)(n2kSources.c_str()), sources, ',',
                         n2kSources.length(), MAX_SOURCES);
    for (int i = n_sources; i < MAX_SOURCES; i++) {
      sources[i] = -1;
    }
  }
  uint16_t sailsAvailable = 0;
  for (int i = 0; i < tState::N_SAILS; i++) {
    if (request->hasArg(("sail" + String(i)).c_str())) {
      sailsAvailable |= 1 << i;
    } else {
      state->sails[i] = 0; // Not on board, so never set
    }
  }
  state->sailsAvailable = sailsAvailable;
  writePreferences();
  request->redirect(getFullUri("index.html"));
}

void handleRestart(AsyncWebServerRequest *request) {
  request->redirect(getFullUri("index.html"));
  scheduleRestart();
}

void handleUpdatePage(AsyncWebServerRequest *request) {
  Serial.println("handleUpdatePage");
  String output =
      "<html><head><title>Firmware Update</title><meta name=\"viewport\" "
      "content=\"width=device-width, initial-scale=1.0\"></head><body>";
  output += "<h1><a href=\"" + getFullUri("index.html") + "\">" + deviceName +
            "</a>/Firmware Update</h1>";
  output += "<form method=\"POST\" action=\"" + getFullUri("update") +
            "\" enctype=\"multipart/form-data\">";
  output += "<input type=\"file\" name=\"update\" accept=\".bin\"> ";
  output += "<input type=\"submit\" value=\"Update\">";
  output += "</form>";
  output += "</body></html>";
  request->send(200, "text/html", output);
}

void handleUpdateUpload(AsyncWebServerRequest *request, String filename,
                        size_t index, uint8_t *data, size_t len, bool final) {
  if (index == 0) {
    Serial.printf("Update: %s\n", filename.c_str());
    // An upload cut midway leaves the previous update open: begin() would
    // then refuse and this image would be appended to the partial one.
    if (Update.isRunning()) {
      Update.abort();
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
    request->onDisconnect([]() {
      if (Update.isRunning()) {
        Serial.println("Update: connection lost, aborting");
        Update.abort();
      }
    });
  }
  if (len > 0 && Update.write(data, len) != len) {
    Update.printError(Serial);
  }
  if (final) {
    if (Update.end(true)) {
      Serial.printf("Update Success: %u bytes\n", (unsigned)(index + len));
    } else {
      Update.printError(Serial);
    }
  }
}

void handleUpdateResult(AsyncWebServerRequest *request) {
  if (Update.hasError()) {
    // The reason goes to the browser too: the serial log may be out of reach
    request->send(200, "text/plain",
                  String("Update FAILED: ") + Update.errorString());
  } else {
    AsyncWebServerResponse *response =
        request->beginResponse(200, "text/plain", "Update OK. Rebooting...");
    response->addHeader("Connection", "close");
    request->send(response);
    scheduleRestart();
  }
}

// ---- Sails API for remote displays ----
// State values as in tState::sails: 0 lowered, 1 full, 2-4 one to three reefs.
static const char *const SAIL_STATE_NAMES[] = {"down", "up", "reef1", "reef2", "reef3"};
static const int N_SAIL_STATES = 5;

// GET /api/sails -> {"sails":[{"id":0,"name":"Main","state":"up"},...]}
// Only the sails on board are listed.
static String sailsJson() {
  String out = "{\"sails\":[";
  bool first = true;
  for (int i = 0; i < tState::N_SAILS; i++) {
    if (!state->hasSail(i)) {
      continue;
    }
    int s = state->sails[i];
    if (s < 0 || s >= N_SAIL_STATES) {
      s = 0;
    }
    if (!first) {
      out += ",";
    }
    first = false;
    out += "{\"id\":" + String(i) + ",\"name\":\"" + tState::SAIL_NAMES[i] +
           "\",\"state\":\"" + SAIL_STATE_NAMES[s] + "\"}";
  }
  out += "]}";
  return out;
}

void handleSailsGet(AsyncWebServerRequest *request) {
  request->send(200, "application/json", sailsJson());
}

// State value (0-4) for a state name if id is a sail on board, -1 otherwise
static int checkSail(int id, const String &value) {
  if (id < 0 || id >= tState::N_SAILS || !state->hasSail(id)) {
    return -1;
  }
  for (int i = 0; i < N_SAIL_STATES; i++) {
    if (value.equalsIgnoreCase(SAIL_STATE_NAMES[i])) {
      return i;
    }
  }
  return -1;
}

// POST /api/sails  sail<id>=<down|up|reef1|reef2|reef3>, one or more
// (form body or query string), e.g. sail0=reef1&sail2=down.
// Only changes the state of sails on board; sails are added or removed in the
// preferences. All or nothing: if any pair is invalid nothing changes.
// Answers with the new status.
void handleSailsPost(AsyncWebServerRequest *request) {
  int values[tState::N_SAILS];  // -1 = leave as is
  for (int i = 0; i < tState::N_SAILS; i++) {
    values[i] = -1;
  }
  bool any = false;

  for (size_t p = 0; p < request->params(); p++) {
    const AsyncWebParameter *param = request->getParam(p);
    const String &key = param->name();
    if (!key.startsWith("sail")) {
      continue;
    }
    String idArg = key.substring(4);
    int id = idArg.toInt();
    if (id == 0 && idArg != "0") {
      id = -1;  // not a number
    }
    int value = checkSail(id, param->value());
    if (value == -1) {
      request->send(400, "text/plain", "bad sail or state: " + key);
      return;
    }
    values[id] = value;
    any = true;
  }
  if (!any) {
    request->send(400, "text/plain", "sail<id>=<state> required");
    return;
  }
  for (int i = 0; i < tState::N_SAILS; i++) {
    if (values[i] != -1) {
      state->sails[i] = values[i];
      Serial.printf("Sail %d -> %s\n", i, SAIL_STATE_NAMES[values[i]]);
    }
  }
  state->sailsDirty = true;
  writeSailState();
  request->send(200, "application/json", sailsJson());
}

void startWebServer() {
  // Called again on every WiFi (re)connection: register the handlers once
  static bool handlersRegistered = false;
  if (!handlersRegistered) {
    handlersRegistered = true;
    server.on("/", HTTP_GET, handleMenu);
    server.on("/index.html", HTTP_GET, handleMenu);
    server.on("/ask", HTTP_GET, handleAskForDelete);
    server.on("/logs", HTTP_GET, handleFileList);
    server.on("/prefs", HTTP_GET, handlePreferences);
    server.on("/updatePrefs", HTTP_POST, handleUpdatePreferences);
    server.on("/clear", HTTP_GET, handleDeleteAll);
    server.on("/help", HTTP_GET, handleHelp);
    server.on("/api/sails", HTTP_GET, handleSailsGet);
    server.on("/api/sails", HTTP_POST, handleSailsPost);
    server.on("/restart", HTTP_GET, handleRestart);
    server.on("/update", HTTP_GET, handleUpdatePage);
    server.on("/update", HTTP_POST, handleUpdateResult, handleUpdateUpload);

    server.onNotFound([](AsyncWebServerRequest *request) {
      if (request->url().startsWith("/del")) {
        deleteFile(request);
      } else {
        handleFileRead(request);
      }
    });
  }

  server.begin();
  Serial.println("HTTP server started");
}
