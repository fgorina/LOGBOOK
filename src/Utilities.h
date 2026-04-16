#pragma once
#include <Arduino.h>
#include <random>
#include <string>

#define NM_TO_METERS 1852.0

bool starts_with(const char* str, const char* pre);
const char* step_into_path(const char* path);
const char* step_into_token(const char* path);
std::string generateUUID();
bool pingServer();
bool uploadFile(const char* sdPath);
