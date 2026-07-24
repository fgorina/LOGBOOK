#pragma once
#include <Arduino.h>
#include <SD.h>

// Writes NMEA2000/TWAI debug output to an SD file instead of Serial, since
// Serial requires a full-power USB cable which conflicts with powering the
// board from an isolated CAN supply. Stops writing after maxLines to avoid
// filling the card if the bus keeps erroring.
class TwaiLog : public Stream {
public:
  void begin(const char *path = "/twai.log", int maxLines = 2000);

  size_t write(uint8_t b) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  int    available() override { return 0; }
  int    read() override { return -1; }
  int    peek() override { return -1; }

private:
  static constexpr int SD_CS = 4; // same SD card/CS as BaroLog

  File _file;
  bool _logEnabled  = false;
  int  _maxLines    = 0;
  int  _loggedLines = 0;
};

extern TwaiLog TwaiLogger;
