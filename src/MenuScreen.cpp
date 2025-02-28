
#include "MenuScreen.h"
#include <M5Tough.h>


MenuScreen::MenuScreen(int width, int height, const char *title) : Screen(width, height, title)
{
   
}
void MenuScreen::enter()
{
wantToClose = false;
   draw();
}
void MenuScreen::exit()
{
    brecord.erase();
    bfiles.erase();
    binspector.erase();
    bweb.erase();
}
void MenuScreen::draw()
{
    M5.Lcd.clear();
    M5.Buttons.draw();
}
bool MenuScreen::run()
{

    if (brecord.event == E_PRESSED)
    {
        // Goto  Record
        Serial.println("Recording");
        
    }
    if (bfiles.event == E_PRESSED)
    {
        // Goto files
        Serial.println("Files");
    }
    if (binspector.event == E_PRESSED)
    {
        // Goto Inspector
        Serial.println("Inspecting");
    }
    if (bweb.event == E_PRESSED)
    {
        // Goto web
        Serial.println("Web Transfer");
    }

    return false;
}