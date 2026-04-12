#include "InfoScreen.h"


InfoScreen::InfoScreen(String *deviceName, String *ssid, String *ip, bool* useN2k, bool* useSK, bool* use0183, String* skServer, int* skPort,  String* sources,  int width, int height, const char *title):Screen (width, height, title)
{
    this->ssid = ssid;
    this->ip = ip;
    this->useN2k = useN2k;
    this->useSK = useSK;
    this->use0183 = use0183;
    this->skServer = skServer;
    this->skPort = skPort;
    this->sources = sources;
    this->deviceName = deviceName;

}

void InfoScreen::enter()

{
    ButtonColors on_clrs = {BLUE, CYAN, WHITE};
    ButtonColors off_clrs = {BLACK, CYAN, WHITE};
    ButtonColors selected_clrs = {RED, WHITE, WHITE};

    bexit = new Button(width / 2 - 100 , height -40, 60, 40, false, "Tornar", off_clrs, on_clrs, MC_DATUM);
    bReboot = new Button(width / 2 + 30 , height -40, 60, 40, false, "Reboot", off_clrs, on_clrs, MC_DATUM);
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
    if (bReboot != nullptr){
        bReboot->delHandlers();
        bReboot->hide(BLACK);
        delete (bReboot);
        bReboot = nullptr;
        Serial.println("Deleted bReboot");
    }
}

void InfoScreen::draw()
{

    
    int pos = 40;
    int delta = 30;

    M5.Display.setFont(&fonts::FreeSans9pt7b);
    Serial.println("InfoScreen::draw");
    M5.Display.clear();

    M5.Display.setTextDatum(TC_DATUM);
    M5.Display.drawString(*deviceName, width / 2, 10);

    M5.Display.setTextDatum(BL_DATUM);
    M5.Display.drawString("SSID: " + *ssid , 10, pos+=delta);

    M5.Display.drawString("IP: " + *ip , 10, pos+=delta);
    M5.Display.drawString("N2K " + String(*useN2k ? "Si" : "No") + "  SK " + String(*useSK ? "Si" : "No") + "  0183 " + String(*use0183 ? "Si" : "No"), 10, pos+=delta);
    M5.Display.drawString("(" + *sources + ")" , 10, pos+=delta);
    
    M5.Display.drawString("SK " + *skServer + " :" + *skPort, 10, pos+=delta);

    M5.Display.setTextDatum(CC_DATUM);
    bexit->draw();
    bDevices->draw();
    bReboot->draw();
}

int InfoScreen::run(const m5::touch_detail_t &t)
{
    if (bDevices != nullptr && bDevices->handleTouch(t))
    {
        return (4);
    }
    if (bexit != nullptr && bexit->handleTouch(t))
    {
        return (0);
    }

    if(bReboot != nullptr && bReboot->handleTouch(t)){
          ESP.restart();
    }
    return -1;
}