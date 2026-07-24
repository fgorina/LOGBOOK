#include "TwaiLog.h"

TwaiLog TwaiLogger;

void TwaiLog::begin(const char *path, int maxLines) {
  _maxLines    = maxLines;
  _loggedLines = 0;
  SD.begin(SD_CS); // no-op if BaroLog already mounted the card
  _file = SD.open(path, FILE_APPEND);
  _logEnabled = (bool)_file;
}

size_t TwaiLog::write(uint8_t b) {
  return write(&b, 1);
}

size_t TwaiLog::write(const uint8_t *buffer, size_t size) {
  if (!_logEnabled) return 0;
  size_t written = _file.write(buffer, size);
  for (size_t i = 0; i < size && _logEnabled; i++) {
    if (buffer[i] != '\n') continue;
    _loggedLines++;
    if (_loggedLines >= _maxLines) {
      _file.close();
      _logEnabled = false;
    }
  }
  if (_logEnabled) _file.flush();
  return written;
}
