#include "Arduino.h"
#include <N2kMessages.h>

#include "N2KDevices.h"
#include "Constants.h"

extern int sources[MAX_SOURCES];
extern int n_sources;
extern void writePreferences();

N2KDevices::N2KDevices(tN2kDeviceList *deviceList, int width, int height,
                       const char *title)
    : Screen(width, height, title) {
  this->deviceList = deviceList;
}

void N2KDevices::enter()

{
  ButtonColors on_clrs = {BLUE, CYAN, WHITE};
  ButtonColors off_clrs = {BLACK, CYAN, WHITE};
  ButtonColors selected_clrs = {RED, WHITE, WHITE};

  bexit = new Button(width / 2 - 30, height - 40, 60, 40, false, "Tornar",
                     off_clrs, on_clrs, MC_DATUM);

  draw();
  // draw(); esta dintre de StartRecord
}

void N2KDevices::exit() {
  Serial.println("N2KDevices::exit");
  if (bexit != nullptr) {
    bexit->delHandlers();
    bexit->hide(BLACK);
    delete (bexit);
    bexit = nullptr;
    Serial.println("Deleted bexit");
  }
}

void N2KDevices::draw() {
  int pos = FIRST_ROW;

  M5.Display.setFont(&fonts::FreeSans9pt7b);
  Serial.println("N2KDevices::draw");
  M5.Display.clear();

  M5.Display.setTextDatum(TC_DATUM);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.drawString("Devices", width / 2, 10);

  M5.Display.setTextDatum(BL_DATUM);
  displayedSources.clear();

  if (deviceList && printDevices) {
    for (uint8_t i = 0; i < N2kMaxBusDevices; i++) {
      tNMEA2000::tDevice *pDevice =
          ((tNMEA2000::tDevice *)deviceList->FindDeviceBySource(i));

      if (pDevice != nullptr && pos < height - 40) {
        uint8_t src = pDevice->GetSource();
        displayedSources.push_back(src);

        uint16_t color = isListened(src) ? TFT_CYAN : TFT_WHITE;
        M5.Display.setTextColor(color, TFT_BLACK);
        M5.Display.drawString(String(src), 10, pos);
        M5.Display.drawString(pDevice->GetModelID(), 60, pos);
        pos += ROW_DELTA;
      }
    }
  }

  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(CC_DATUM);
  bexit->draw();
}

bool N2KDevices::isListened(uint8_t src) {
  for (int i = 0; i < n_sources; i++)
    if (sources[i] == src) return true;
  return false;
}

void N2KDevices::toggleSource(uint8_t src) {
  for (int i = 0; i < n_sources; i++) {
    if (sources[i] == (int)src) {
      for (int j = i; j < n_sources - 1; j++) sources[j] = sources[j + 1];
      sources[--n_sources] = -1;
      writePreferences();
      return;
    }
  }
  if (n_sources < MAX_SOURCES) {
    sources[n_sources++] = src;
    writePreferences();
  }
}

int N2KDevices::run(const m5::touch_detail_t &t) {
  if (bexit != nullptr && bexit->handleTouch(t)) {
    return (0);
  }

  if (t.wasClicked()) {
    int row = (t.y - FIRST_ROW + ROW_DELTA) / ROW_DELTA - 1;
    if (row >= 0 && row < (int)displayedSources.size()) {
      toggleSource(displayedSources[row]);
      draw();
    }
  }

  if (deviceList && deviceList->ReadResetIsListUpdated()) {
    printDevices = true;
    draw();
  }
  return -1;
}