/* Screen Prototype */

#ifndef _MENUScreen_H_
#define _MENUScreen_H_
#include <M5Tough.h>
#include "Screen.h"

#define  MAXFILES 100
#define  MAXNAME  11 + 5
#define HEADER_SIZE  30
#define  FILES_SCREEN  7

class MenuScreen : public Screen
{   
    public:
    MenuScreen(int width, int height, const char* title);
    void enter();
    void exit();
    void draw();
    bool run();

    protected: 
    
   
    ButtonColors on_clrs = {BLUE, CYAN, WHITE};
    ButtonColors off_clrs = {BLACK, CYAN, WHITE};
    ButtonColors selected_clrs = {RED, WHITE, WHITE};
    Button brecord = Button(width /4, 30, width / 2, 50, false, "Gravar", off_clrs, on_clrs, MC_DATUM);
    Button bfiles = Button(width / 4, 90, width / 2, 50, false, "Logs", off_clrs, on_clrs, MC_DATUM);
    Button binspector = Button(width / 4, 150, width / 2, 50, false, "Inspector", off_clrs, on_clrs, MC_DATUM);
    Button bweb = Button(width / 4, 210, width / 2, 50, false, "Web", off_clrs, on_clrs, MC_DATUM);



};

#endif