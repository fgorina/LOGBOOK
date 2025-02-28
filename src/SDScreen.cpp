#include "SDScreen.h"
#include <M5Tough.h>

SDScreen::SDScreen(int width, int height, const char *title) : Screen(width, height, title)
{
   
}
void SDScreen::enter()
{
    wantToClose = false;
   draw();
}
void SDScreen::exit()
{
    brecord.erase();
    bup.erase();
    bdown.erase();
}
void SDScreen::draw()
{
    Serial.println("Drawing Screen");
    drawSD();
    M5.Buttons.draw();
}
bool SDScreen::run()
{

    if (M5.Touch.changed && M5.Touch.point[0].x > width/2 && M5.Touch.point[0].x < width)
    {
        do_select_file();
    }
    if (brecord.event == E_PRESSED)
    {
        wantToClose = true;
    }
    if (bup.event == E_PRESSED)
    {
        first_file = max(first_file - FILES_SCREEN, 0);
        drawSD();
    }
    if (bdown.event == E_PRESSED)
    {
        first_file = min(first_file + FILES_SCREEN, max(n_files - FILES_SCREEN, 0));
        drawSD();  
    }


    return wantToClose;
}

void SDScreen::loadSD()
{
  File root = SD.open("/");
  n_files = 0;
  if (root)
  {

    while (true)
    {
      File entry = root.openNextFile();
      if (!entry)
      {
        // no more files
        break;
      }

      if (entry.name()[0] != '.')
      {
        Serial.println(entry.name());
        strcpy(sd_files[n_files], entry.name());
        n_files++;
        entry.close();
      }
    }
    root.close();
    sortFiles(); // Sort the files after loading them
  }
}
void SDScreen::drawFile(int file, bool selected)
{
    if (file >= 0 && file < n_files && file >= first_file && file < first_file + FILES_SCREEN)
    {

        M5.Lcd.setTextDatum(CL_DATUM);
        if (selected)
        {
            M5.Lcd.setTextColor(BLACK);
            M5.Lcd.fillRect(0, (file - first_file) * 30 + HEADER_SIZE, width / 2, 30, WHITE);
            M5.Lcd.drawString(sd_files[file], 10, HEADER_SIZE + 15 + (file - first_file) * 30);
        }
        else
        {
            M5.Lcd.setTextColor(WHITE);
            M5.Lcd.fillRect(0, (file - first_file) * 30 + HEADER_SIZE, width / 2, 30, BLACK);
            M5.Lcd.drawString(sd_files[file], 10, HEADER_SIZE + 15 + (file - first_file) * 30);
        }
    }
}
void SDScreen::drawSD()
{

    M5.Lcd.fillRect(0, 0, width / 2, height, BLACK);
    M5.Lcd.setTextDatum(CC_DATUM);

    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.fillRect(0, 0, width / 2, HEADER_SIZE, BLUE);
    M5.Lcd.drawString("Registres", width / 4, HEADER_SIZE / 2);

    for (int i = first_file; i < n_files && i < first_file + FILES_SCREEN; i++)
    {
        drawFile(i, i == selected_file);
    }
}

void SDScreen::sortFiles()
{
    auto compareStrings = [](const void *a, const void *b)
    {
        return strcmp((const char *)b, (const char *)a);
    };
    qsort(sd_files, n_files, MAXNAME, compareStrings);
}


void SDScreen::do_select_file()
{

  // Now compute the line
  if (M5.Touch.point[0].x >= 0 && M5.Touch.point[0].y >= HEADER_SIZE)
  {
    int line = floor((M5.Touch.point[0].y - HEADER_SIZE) / 30) + first_file;
    if (line < n_files)
    {
      Serial.print("Clicked on file ");
      Serial.print(line);
      Serial.print(" : ");
      Serial.println(sd_files[line]);
      drawFile(selected_file, false);
      selected_file = line;
      drawFile(selected_file, true);
    }
    else
    {
      Serial.print("Clicked outside file ");
      Serial.println(line);
      drawFile(selected_file, false);
      selected_file = -1;
    }
  }
}