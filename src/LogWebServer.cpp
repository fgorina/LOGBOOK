/* Configuration web server

    Pages served on http://<deviceName>.local/ : menu, preferences, logs
    (list, download, delete), help, restart and firmware update.
*/

#include "LogWebServer.h"

#include <Arduino.h>
#include <SD.h>
#include <SPIFFS.h>
#include <Update.h>
#include <WebServer.h>

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

void writePreferences();

static WebServer server(80);

void handleWebServer() { server.handleClient(); }


String getContentType(String filename) {
  if (server.hasArg("download")) {
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

String getFullUri(String last) {
  return "http://" + deviceName + ".local/" + last;
}

void handleHelp() {
  Serial.println("handleHelp");
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  File file = SPIFFS.open("/help.html", "r");
  if (!file) {
    Serial.println("File not found");
  } else {
    server.streamFile(file, "text/html");
    file.close();
  }
  SPIFFS.end();
}
bool handleFileRead(String spath) {
  String path = spath;
  Serial.printf("Downloading %s\n", path.c_str());
  if (path.endsWith("/")) {
    path += "index.htm";
  }

  if (!SD.exists(path)) {
    Serial.println("File " + path + " not found.");
    server.send(404, "text/plain", "FileNotFound");
    return false;
  }

  Serial.println("handleFileRead: " + path);
  String contentType = getContentType(path);

  File file = SD.open(path, FILE_READ);
  if (file) {
    server.streamFile(file, contentType);
    file.close();
    return true;
  } else {
    Serial.println("File " + path + " not found.");
    return false;
  }
}

void handleFileList() {
  Serial.println("handleFileList");

  // Stream in chunks: avoids building a large String in heap and lets the
  // browser receive data progressively without waiting for the full page.
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  server.sendContent("<html><head><title>Logs</title>"
                     "<meta name=\"viewport\" content=\"width=device-width, "
                     "initial-scale=1.0\">"
                     "</head><body>\n");
  server.sendContent("<h1><a href=\"" + getFullUri("index.html") + "\">" +
                     deviceName + "</a>/Logs</h1>\n");
  server.sendContent("<a href=\"" + getFullUri("ask") +
                     "\">Esborrar tots els Logs</a><br>\n");
  server.sendContent("<ul>\n");

  File root = SD.open("/logs");
  if (root.isDirectory()) {
    File file = root.openNextFile();
    while (file) {
      if (file.name()[0] != '.') {
        String path = file.path();
        server.sendContent("<li><a href=\"" + getFullUri(path) + "\">" +
                           file.name() +
                           "</a>&nbsp;&nbsp;"
                           "<a href=\"" +
                           getFullUri("del/" + path) + "\">Delete</a></li>\n");
      }
      file = root.openNextFile();
    }
    root.close();
  }

  server.sendContent("</ul></body></html>\n");
}

void handleMenu() {
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
  output += "</body></html>";
  unsigned long len = output.length();
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);
}

void handleAskForDelete() {
  Serial.println("handleAskForDelete");
  String output =
      "<html><head><title>Confirmeu, si us plau</title><meta name=\"viewport\" "
      "content=\"width=device-width, initial-scale=1.0</head><body>";
  output +=
      "Segur que voleu esborrar tots els logs? <a href=" + getFullUri("clear") +
      ">Si</a> <a href=" + getFullUri("logs") + ">No</a>";
  unsigned long len = output.length();
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);
}
void handleDeleteAll() {
  Serial.println("handleDeleteAll");

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

  output += "</ul>\n";
  server.send(200, "text/html", output);
}

void deleteFile(String uri) {
  Serial.println("deleteFile uri " + uri);
  String path = uri.substring(4, uri.length());
  Serial.println("deleteFile " + path);
  SD.remove(path);
  handleFileList();
}

void handlePreferences() {
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
  unsigned long len = output.length();
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);
}

void handleUpdatePreferences() {

  Serial.println("handleUpdatePreferences");
  if (server.hasArg("ssid")) {
    wifi_ssid = server.arg("ssid");
  }
  if (server.hasArg("password")) {
    wifi_password = server.arg("password");
  }
  if (server.hasArg("skserver")) {
    skServer = server.arg("skserver");
  }
  if (server.hasArg("skport")) {
    skPort = server.arg("skport").toInt();
  }
  if (server.hasArg("usexml")) {
    ((RecordScreen *)screens[1])->xmlFormat = true;
  } else {
    ((RecordScreen *)screens[1])->xmlFormat = false;
  }

  if (server.hasArg("usen2k")) {
    useN2k = true;
  } else {
    useN2k = false;
  }
  if (server.hasArg("usesk")) {
    useSK = true;
  } else {
    useSK = false;
  }
  use0183 = server.hasArg("use0183");
  if (server.hasArg("n2kdevices")) {
    n2kSources = server.arg("n2kdevices");
    n_sources = splitter((char *)(n2kSources.c_str()), sources, ',',
                         n2kSources.length(), MAX_SOURCES);
    for (int i = n_sources; i < MAX_SOURCES; i++) {
      sources[i] = -1;
    }
  }
  uint16_t sailsAvailable = 0;
  for (int i = 0; i < tState::N_SAILS; i++) {
    if (server.hasArg("sail" + String(i))) {
      sailsAvailable |= 1 << i;
    } else {
      state->sails[i] = 0; // Not on board, so never set
    }
  }
  state->sailsAvailable = sailsAvailable;
  writePreferences();
  server.sendHeader("Location", getFullUri("index.html"), true);
  server.send(302, "text/plain", "");
}

void handleRestart() {
  server.sendHeader("Location", getFullUri("index.html"), true);
  server.send(302, "text/plain", "");
  Serial.println("Restarting");
  ESP.restart();
}

void handleUpdatePage() {
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
  unsigned long len = output.length();
  server.sendHeader("Content-Length", String(len));
  server.send(200, "text/html", output);
}

void handleUpdateUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Update: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("Update Success: %u bytes\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
}

void handleUpdateResult() {
  if (Update.hasError()) {
    server.send(200, "text/plain", "Update FAILED. Check serial log.");
  } else {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "Update OK. Rebooting...");
    delay(500);
    ESP.restart();
  }
}

void startWebServer() {
  server.on("/", HTTP_GET, handleMenu);
  server.on("/index.html", HTTP_GET, handleMenu);
  server.on("/ask", HTTP_GET, handleAskForDelete);
  server.on("/logs", HTTP_GET, handleFileList);
  server.on("/prefs", HTTP_GET, handlePreferences);
  server.on("/updatePrefs", HTTP_POST, handleUpdatePreferences);
  server.on("/clear", HTTP_GET, handleDeleteAll);
  server.on("/help", HTTP_GET, handleHelp);
  server.on("/restart", HTTP_GET, handleRestart);
  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/update", HTTP_POST, handleUpdateResult, handleUpdateUpload);

  server.onNotFound([]() {
    if (server.uri().startsWith("/del")) {
      deleteFile(server.uri());
    } else if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "FileNotFound");
    } else {
      Serial.printf("Not Found %s\n", server.uri().c_str());
    }
  });

  server.begin();
  Serial.println("HTTP server started");
}
