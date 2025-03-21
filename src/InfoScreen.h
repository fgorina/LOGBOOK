/* Screen Prototype */

#ifndef _InfoScreen_H_
#define _InfoScreen_H_

#include "Screen.h"

#define HEADER_SIZE 30

class InfoScreen : public Screen
{
public:
    InfoScreen(String ssid, String *ip, bool* useN2k, bool* useSK, String* skServer, int* skPort,  String* sources, int width, int height, const char *title);

    void enter();
    void exit();
    void draw();
    int run();

protected:
    Button *bexit = nullptr;
    Button *bDevices = nullptr;

    String ssid;

    String *ip;

    bool* useN2k;
    bool* useSK;
    String* skServer;
    int* skPort;
    String* sources;

};

#endif
