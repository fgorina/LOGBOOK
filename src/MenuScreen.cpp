
#include "MenuScreen.h"
#include <M5Tough.h>
#include "Arduino.h"
#include <WiFi.h>



MenuScreen::MenuScreen(int width, int height, const char *title) : Screen(width, height, title)
{
    Serial.println("MenuScreen::MenuScreen");
}

MenuScreen::~MenuScreen()
{
    Serial.println("MenuScreen::~MenuScreen");
}

void MenuScreen::enter()
{
    Serial.println("MenuScreen::enter");
    ButtonColors on_clrs = {BLUE, CYAN, WHITE};
    ButtonColors off_clrs = {BLACK, CYAN, WHITE};
    ButtonColors selected_clrs = {RED, WHITE, WHITE};

    brecord = new Button(width / 4, 50, width / 2, 50, false, "Gravar", off_clrs, on_clrs, MC_DATUM);
    bfiles = new Button(width / 4, 110, width / 2, 50, false, "Logs", off_clrs, on_clrs, MC_DATUM);
    binspector = new Button(width / 4, 170, width / 2, 50, false, "Inspector", off_clrs, on_clrs, MC_DATUM);
   

    draw();
}
void MenuScreen::exit()
{
    Button *button;

    Serial.println("MenuScreen::exit");
    if (brecord != nullptr)
    {
        brecord->delHandlers();
        brecord->hide(BLACK);
        delete (brecord);
        brecord = nullptr;
    }

    if (bfiles != nullptr)
    {
        bfiles->delHandlers();
        bfiles->hide(BLACK);
        delete (bfiles);
        bfiles = nullptr;
    }

    if (binspector != nullptr)
    {
        binspector->delHandlers();
        binspector->hide(BLACK);
        delete (binspector);
        binspector = nullptr;
    }

  
}
void MenuScreen::draw()
{
    Serial.println("MenuScreen::draw");
    M5.Lcd.clear();

    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString( WiFi.localIP().toString(), width / 2, 10);
    if (brecord != nullptr && bfiles != nullptr && binspector != nullptr)
    {
        brecord->draw();
        bfiles->draw();
        binspector->draw();
    }
    // M5.Buttons.draw();
    Serial.println("Exit MenuScreen::draw");
}
int MenuScreen::run()
{

    if (brecord != nullptr && brecord->wasReleased())
    {
        // Goto  Record
        Serial.println("Record");
        return (1);
    }
    if (bfiles != nullptr && bfiles->wasReleased())
    {
        // Goto files
        Serial.println("Files");
        return (2);
    }
    if (binspector != nullptr && binspector->wasReleased())
    {
        // Goto Inspector
        Serial.println("Inspecting");
        return (3);
    }
    

    return -1;
}