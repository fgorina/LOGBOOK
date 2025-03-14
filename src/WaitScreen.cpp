#include "WaitScreen.h"

WaitScreen::WaitScreen(int width, int height, const char *title) : Screen(width, height, title)
{
    
}

void WaitScreen::enter()
{
    draw();
   // draw(); esta dintre de StartRecord
}

void WaitScreen::exit()
{
    Serial.println("WaitScreen::exit");

}

void WaitScreen::draw()
{
    M5.Lcd.setFreeFont(&FreeSans9pt7b);
    Serial.println("WaitScreen::draw");
    M5.Lcd.clear();

    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("Logbook", width/2, 10);

    M5.Lcd.setTextDatum(CC_DATUM);
    M5.Lcd.drawString("Feu servir un navegador", width/2, 80);
    M5.Lcd.drawString("per configurar Logbook", width/2, 120);
    M5.Lcd.drawString("ssid: logbook / passws: 123456", width/2, 160);

    M5.Lcd.drawString("url: http://logbook.local", width/2, 180);


}

int WaitScreen::run(){
    return -1;
}