/* Screen Prototype */

#ifndef _InfoScreen_H_
#define _InfoScreen_H_

#include "Screen.h"

#define HEADER_SIZE 30

class InfoScreen : public Screen
{
public:
    InfoScreen(String *deviceName, String *ssid, String *ip, bool* useN2k, bool* useSK, bool* use0183, String* skServer, int* skPort,  String* sources, int width, int height, const char *title);

    void enter() override;
    void exit() override;
    void draw() override;
    int run(const m5::touch_detail_t &t) override;

protected:
    Button *bexit = nullptr;
    Button *bDevices = nullptr;
    Button *bReboot = nullptr;

    String *ssid;

    String *ip;

    String* deviceName;
    bool* useN2k;
    bool* useSK;
    bool* use0183;
    String* skServer;
    int* skPort;
    String* sources;

};

#endif
