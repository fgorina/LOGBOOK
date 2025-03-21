/* Screen Prototype */

#ifndef _MENUScreen_H_
#define _MENUScreen_H_
#include <M5Tough.h>
#include "Screen.h"
#include "State.h"



#define HEADER_SIZE  30


class MenuScreen : public Screen
{   
    public:
    MenuScreen(tState* state, int width, int height, const char* title);
    ~MenuScreen();
    void enter();
    void exit();
    void draw();
    int run();

    protected: 
    tState* state;
    Button *brecord = nullptr;
    Button *bfiles= nullptr;
    Button *binspector = nullptr;


};

#endif