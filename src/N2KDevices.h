/* Screen Prototype */

#pragma once

#include "Screen.h"
#include "N2kDeviceList.h"
#include <vector>

#define HEADER_SIZE 30

class N2KDevices : public Screen
{
public:
    N2KDevices(tN2kDeviceList* deviceList, int width, int height, const char *title);

    void enter() override;
    void exit() override;
    void draw() override;
    int run(const m5::touch_detail_t &t) override;

protected:
    static const int FIRST_ROW = 40;
    static const int ROW_DELTA = 30;

    Button *bexit = nullptr;

    tN2kDeviceList* deviceList;
    bool printDevices = false;

    std::vector<uint8_t> displayedSources;

    bool isListened(uint8_t src);
    void toggleSource(uint8_t src);
};