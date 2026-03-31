#include "Arduino.h"
#include <N2kMessages.h>

#include "N2KDevices.h"
#include "Constants.h"

extern int sources[MAX_SOURCES];
extern int n_sources;

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
  int pos = 40;
  int delta = 30;

  M5.Display.setFont(&fonts::FreeSans9pt7b);
  Serial.println("N2KDevices::draw");
  M5.Display.clear();

  M5.Display.setTextDatum(TC_DATUM);
  M5.Display.drawString("Devices", width / 2, 10);

  M5.Display.setTextDatum(BL_DATUM);

  if (deviceList) {

    Serial.println("Devices " + N2kMaxBusDevices);

    if (printDevices) {
      for (uint8_t i = 0; i < N2kMaxBusDevices; i++) {

        Serial.println("Device " + i);

        tNMEA2000::tDevice *pDevice =
            ((tNMEA2000::tDevice *)deviceList->FindDeviceBySource(i));

        if (pDevice != 0) {

          if (pos < height) {
            M5.Display.drawString(String(pDevice->GetSource()), 10, pos);
            M5.Display.drawString(pDevice->GetModelID(), 100, pos += delta);
          }
        }
      }
    }
  }

  M5.Display.setTextDatum(CC_DATUM);
  bexit->draw();
}

int N2KDevices::run(const m5::touch_detail_t &t) {
  if (bexit != nullptr && bexit->handleTouch(t)) {
    return (0);
  }

  if (deviceList && deviceList->ReadResetIsListUpdated()) {
    printDevices = true;
    draw();
  }
  return -1;
}