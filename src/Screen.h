/* Screen Prototype */

#ifndef _Screen_H_
#define _Screen_H_

#include <M5Tough.h>


class Screen
{
    protected:
        int width;
        int height;
        const char* title;

        bool wantToClose = false;


 
    public:
    Screen(int width, int height, const char* title);
    
    void enter();
    void exit();
    void draw();
    bool run();

};



#endif