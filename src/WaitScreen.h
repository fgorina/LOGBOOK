/* Screen Prototype */

#ifndef _WaitScreen_H_
#define _Waitcreen_H_

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

#endif

