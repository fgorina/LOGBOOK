#ifndef UTILITIES_H
#define UTILITIES_H
#include <Arduino.h>
#ifdef __cplusplus
extern "C" {
#endif

#define NM_TO_METERS 1852.0

bool starts_with(const char* str, const char* pre) {
    return strncmp(pre, str, strlen(pre)) == 0;
  }

  const char* step_into_path(const char* path) {
    if (path == NULL) return NULL;
    const char* s = strchr(path, '/');
    return s != NULL ? ++s : NULL;
  }

  const char* step_into_token(const char* path) {
    if (path == NULL) return NULL;
    const char* s = strchr(path, '.');
    return s != NULL ? ++s : NULL;
  }

  #ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
