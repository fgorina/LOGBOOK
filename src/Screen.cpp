#include "Screen.h"



Screen::Screen(int width, int height, const char* title){  
    this->width = width;
    this->height = height;
    this->title = title;

}

void Screen::enter()
{

        
    draw();
}
void Screen::exit()
{


}

void Screen::draw()
{

}
int Screen::run()
{
    Serial.println("Screen::run");
    return -1;
}