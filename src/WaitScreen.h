/* Screen Prototype */

#pragma once

#include "Screen.h"



#define HEADER_SIZE  30


class WaitScreen : public Screen
{   
    public:
    WaitScreen(int width, int height, const char* title);

    void enter();
    void exit();
    void draw();
    int run();


};

