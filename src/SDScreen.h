/* Screen Prototype */

#ifndef _SDScreen_H_
#define _SDScreen_H_
#include <M5Tough.h>
#include "Screen.h"

#define  MAXFILES 100
#define  MAXNAME  11 + 5
#define HEADER_SIZE  30
#define  FILES_SCREEN  7

class SDScreen : public Screen
{   
    public:
    SDScreen(int width, int height, const char* title);
    void enter();
    void exit();
    void draw();
    bool run();

    protected: 
    
    char sd_files[MAXFILES][MAXNAME];
    int n_files = 0;
    int first_file = 0;
    int selected_file = -1; 

    ButtonColors on_clrs = {BLUE, CYAN, WHITE};
    ButtonColors off_clrs = {BLACK, CYAN, WHITE};
    ButtonColors selected_clrs = {RED, WHITE, WHITE};
    Button brecord = Button(width / 2 + width / 8, height / 2 - 30, width / 4, 60, false, "Start", off_clrs, on_clrs, MC_DATUM);
    Button bup = Button(width / 2 + width / 8, 10, width / 4, 60, false, "^", off_clrs, on_clrs, MC_DATUM);
    Button bdown = Button(width / 2 + width / 8, height - 70, width / 4, 60, false, "v", off_clrs, on_clrs, MC_DATUM); 


    void sortFiles();
    void loadSD();
    void drawFile(int file, bool selected);
    void drawSD();
    void do_select_file();

};

#endif