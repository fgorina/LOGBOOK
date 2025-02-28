#include "SDScreen.h"
#include <M5Tough.h>

SDScreen::SDScreen(int width, int height, const char *title) : Screen(width, height, title)
{
  
}
void SDScreen::enter()
{
  Serial.println("SDScreen::enter");
  loadSD();
  ButtonColors on_clrs = {BLUE, CYAN, WHITE};
  ButtonColors off_clrs = {BLACK, CYAN, WHITE};
  ButtonColors selected_clrs = {RED, WHITE, WHITE};

  brecord = new Button(width / 2 + width / 8, height / 2 - 30, width / 4, 60, false, "Exit", off_clrs, on_clrs, MC_DATUM);
  bup = new Button(width / 2 + width / 8, 10, width / 4, 60, false, "^", off_clrs, on_clrs, MC_DATUM);
  bdown = new Button(width / 2 + width / 8, height - 70, width / 4, 60, false, "v", off_clrs, on_clrs, MC_DATUM);

  draw();
}
void SDScreen::exit()
{
  Serial.println("SDScreen::exit");

  if (brecord != nullptr)
  {
    brecord->delHandlers();
    brecord->hide(BLACK);
    delete (brecord);
    brecord = nullptr;
    Serial.println("Deleted brecord");
  }

  if (bup != nullptr)
  {
    bup->delHandlers();
    bup->hide(BLACK);
    delete (bup);
    bup = nullptr;
    Serial.println("Deleted bup");
  }

  if (bdown != nullptr)
  {
    bdown->delHandlers();
    bdown->hide(BLACK);
    delete (bdown);
    bdown = nullptr;
    Serial.println("Deleted bdown");
  }
}
void SDScreen::draw()
{

  Serial.println("SDScreen::draw");
  M5.Lcd.clear();
  drawSD();
  brecord->draw();
  bup->draw();
  bdown->draw();
}
int SDScreen::run()
{

  if (M5.Touch.changed && M5.Touch.point[0].x < width / 2 && M5.Touch.point[0].x > 0)
  {
    do_select_file();
  }
  if (brecord != nullptr && brecord->wasPressed())
  {
    return (0);
  }
  if (bup != nullptr && bup->wasPressed())
  {
    first_file = max(first_file - FILES_SCREEN, 0);
    drawSD();
  }
  if (bdown != nullptr && bdown->wasPressed())
  {
    first_file = min(first_file + FILES_SCREEN, max(n_files - FILES_SCREEN, 0));
    drawSD();
  }

  return -1;
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
        int l = strlen(entry.name());
        strncpy(sd_files[n_files], entry.name(), MAXNAME - 1);
        sd_files[n_files][sizeof(sd_files[n_files]) - 1] = '\0'; // Ensure null-termination
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