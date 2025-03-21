/* Screen Prototype */

#ifndef _N2KDevices_H_
#define _N2KDevices_H_

#include "Screen.h"
#include "N2kDeviceList.h"

#define HEADER_SIZE 30

class N2KDevices : public Screen
{
public:
    N2KDevices(tN2kDeviceList* deviceList, int width, int height, const char *title);

    void enter();
    void exit();
    void draw();
    int run();

protected:
    Button *bexit = nullptr;

    tN2kDeviceList* deviceList;
    bool printDevices = false;

};

#endif