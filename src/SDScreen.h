/* Screen Prototype */

#ifndef _SDScreen_H_
#define _SDScreen_H_

#include "Screen.h"
#include "State.h"

#define  MAXFILES 100
#define  MAXNAME 20
#define HEADER_SIZE  30
#define  FILES_SCREEN  7

class SDScreen : public Screen
{   
    public:
    SDScreen(int width, int height, const char* title, tState* state);
    void enter();
    void exit();
    void draw();
    int run();
    bool exists(const char* name);
    
    protected: 
    
    char sd_files[MAXFILES][MAXNAME];
    int n_files = 0;
    int first_file = 0;
    int selected_file = -1; 

    Button *bexit = nullptr;
    Button *bup = nullptr;
    Button *bdown = nullptr; 
    tState* state;

    void sortFiles();
    void loadSD();
    void drawFile(int file, bool selected);
    void drawSD();
    void do_select_file();
    


};

#endif