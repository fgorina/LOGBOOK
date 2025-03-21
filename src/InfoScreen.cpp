#include "InfoScreen.h"

InfoScreen::InfoScreen(String ssid, String *ip, bool* useN2k, bool* useSK, String* skServer, int* skPort,  String* sources,  int width, int height, const char *title):Screen (width, height, title)
{
    this->ssid = ssid;
    this->ip = ip;
    this->useN2k = useN2k;
    this->useSK = useSK;
    this->skServer = skServer;
    this->skPort = skPort;
    this->sources = sources;

}

void InfoScreen::enter()

{
    ButtonColors on_clrs = {BLUE, CYAN, WHITE};
    ButtonColors off_clrs = {BLACK, CYAN, WHITE};
    ButtonColors selected_clrs = {RED, WHITE, WHITE};

    bexit = new Button(width / 2 - 30 , height -40, 60, 40, false, "Tornar", off_clrs, on_clrs, MC_DATUM);
    bDevices = new Button(width-70 , 110, 60, 40, false, "Devs", off_clrs, on_clrs, MC_DATUM);

    draw();
    // draw(); esta dintre de StartRecord
}

void InfoScreen::exit()
{
    Serial.println("InfoScreen::exit");
    if (bexit != nullptr)
    {
        bexit->delHandlers();
        bexit->hide(BLACK);
        delete (bexit);
        bexit = nullptr;
        Serial.println("Deleted bexit");
    }
    if (bDevices != nullptr){
        bDevices->delHandlers();
        bDevices->hide(BLACK);
        delete (bDevices);
        bDevices = nullptr;
        Serial.println("Deleted bDevices");
    }
}

void InfoScreen::draw()
{
    int pos = 40;
    int delta = 30;

    M5.Lcd.setFreeFont(&FreeSans9pt7b);
    Serial.println("InfoScreen::draw");
    M5.Lcd.clear();

    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("Logbook", width / 2, 10);

    M5.Lcd.setTextDatum(BL_DATUM);
    M5.Lcd.drawString("SSID: " + ssid , 10, pos+=delta);

    M5.Lcd.drawString("IP: " + *ip , 10, pos+=delta);
    M5.Lcd.drawString("N2K " + String(*useN2k ? "Si" : "No") + "     SK " + String(*useSK ? "Si" : "No") , 10, pos+=delta);
    M5.Lcd.drawString("(" + *sources + ")" , 10, pos+=delta);
    M5.Lcd.drawString("SK " + *skServer + " :" + *skPort, 10, pos+=delta);

    M5.Lcd.setTextDatum(CC_DATUM);
    bexit->draw();
    bDevices->draw();
}

int InfoScreen::run()
{
    if (bDevices != nullptr && bDevices->wasReleased())
    {
        return (4);
    }
    if (bexit != nullptr && bexit->wasReleased())
    {
        return (0);
    }
    return -1;
}