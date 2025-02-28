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

    public:
    Screen(int width, int height, const char* title);
    
    virtual void enter();
    virtual void exit();
    virtual void draw();
    virtual int run();

};



#endif