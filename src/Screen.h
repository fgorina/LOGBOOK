/* Screen Prototype 

    This is thwe prototype of the Screen classes.alignas

    Screens may have any structure but you must take into account that :alignas

        - They are created just once and stored in an array of screens in main.cpp 
        - When they are going to be shown enter() methos is called. It should create any buttons, call clear and then draw
        - When they are removed exit() is called. You must remove any buttons deleting then and setting the variables to nullptr
        - Every iteration of loop run is called. It may return:alignas
            - -1 Just maintain the same screen
            - 0 Go to the menu screen
            - any number smaller than the number of screens, go to that screen

        - draw() showld draw the screen but is not usually called from outside.

    As each screen my have it's data, if they must access global data it must be passed in the constructor.

    What is very important is that they are not created and destroyed. Just the buttons as there is no other way. If not
    therem are memory problems.

*/

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