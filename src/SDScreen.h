/* Screen Prototype */

#ifndef _SDScreen_H_
#define _SDScreen_H_
#include <M5Tough.h>
#include "Screen.h"

#define  MAXFILES 100
#define  MAXNAME  20
#define HEADER_SIZE  30
#define  FILES_SCREEN  7

class SDScreen : public Screen
{   
    public:
    SDScreen(int width, int height, const char* title);
    void enter();
    void exit();
    void draw();
    int run();

    protected: 
    
    char sd_files[MAXFILES][MAXNAME];
    int n_files = 0;
    int first_file = 0;
    int selected_file = -1; 

    Button *brecord = nullptr;
    Button *bup = nullptr;
    Button *bdown = nullptr; 


    void sortFiles();
    void loadSD();
    void drawFile(int file, bool selected);
    void drawSD();
    void do_select_file();

};

#endif