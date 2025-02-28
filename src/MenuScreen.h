/* Screen Prototype */

#ifndef _MENUScreen_H_
#define _MENUScreen_H_
#include <M5Tough.h>
#include "Screen.h"



#define HEADER_SIZE  30


class MenuScreen : public Screen
{   
    public:
    MenuScreen(int width, int height, const char* title);
    ~MenuScreen();
    void enter();
    void exit();
    void draw();
    int run();

    protected: 
    
    Button *brecord = nullptr;
    Button *bfiles= nullptr;
    Button *binspector = nullptr;
    Button *bweb = nullptr;

};

#endif