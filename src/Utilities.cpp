#include "Utilities.h"
#include <SD.h>
#include <HTTPClient.h>
#include <vector>

extern SemaphoreHandle_t sdMutex;

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

std::string generateUUID()
{
    static std::mt19937 rng(std::random_device{}());
    static std::uniform_int_distribution<int> dist(0, 15);
    const char *hex = "0123456789abcdef";
    std::string uuid = "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx";

    for (char &c : uuid)
    {
        if (c == 'x')
            c = hex[dist(rng)];
        else if (c == 'y')
            c = hex[(dist(rng) & 0x3) | 0x8];
    }
    return uuid;
}

static const char *kServerBase = "http://192.168.1.150:8765";

bool pingServer()
{
    HTTPClient http;
    String url = String(kServerBase) + "/ping";
    http.begin(url);
    http.setTimeout(3000);
    int code = http.GET();
    http.end();
    if (code != 200)
    {
        Serial.printf("[INFO] pingServer: /ping returned %d — server unreachable\n", code);
        return false;
    }
    return true;
}

bool uploadFile(const char *sdPath)
{
    // 0. Check server reachability
    if (!pingServer())
        return false;

    // 1. Generate UUID
    std::string uuid = generateUUID();

    // 2. Read file into buffer while holding the mutex; release before the POST
    std::vector<uint8_t> fileData;
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE)
    {
        File f = SD.open(sdPath, FILE_READ);
        if (!f)
        {
            xSemaphoreGive(sdMutex);
            Serial.printf("[ERROR] uploadFile: cannot open %s\n", sdPath);
            return false;
        }
        size_t sz = f.size();
        fileData.resize(sz);
        f.read(fileData.data(), sz);
        f.close();
        xSemaphoreGive(sdMutex);
    }
    else
    {
        Serial.printf("[ERROR] uploadFile: could not acquire sdMutex\n");
        return false;
    }

    // 3. Build multipart/form-data body entirely in memory (mutex already released)
    const char *basename = strrchr(sdPath, '/');
    basename = (basename != nullptr) ? basename + 1 : sdPath;

    const char *boundary = "----ESP32Boundary7MA4YWxkTrZu0gW";

    String namePart = String("--") + boundary + "\r\n"
                      "Content-Disposition: form-data; name=\"name\"\r\n\r\n"
                    + uuid.c_str() + "\r\n";

    String filePart = String("--") + boundary + "\r\n"
                      "Content-Disposition: form-data; name=\"file\"; filename=\""
                    + basename + "\"\r\n"
                      "Content-Type: application/octet-stream\r\n\r\n";

    String trailer = String("\r\n--") + boundary + "--\r\n";

    std::vector<uint8_t> body;
    body.reserve(namePart.length() + filePart.length() + fileData.size() + trailer.length());

    for (size_t i = 0; i < namePart.length(); i++) body.push_back((uint8_t)namePart[i]);
    for (size_t i = 0; i < filePart.length(); i++) body.push_back((uint8_t)filePart[i]);
    body.insert(body.end(), fileData.begin(), fileData.end());
    for (size_t i = 0; i < trailer.length();  i++) body.push_back((uint8_t)trailer[i]);

    // 4. POST — blocking call
    HTTPClient http;
    http.begin(String(kServerBase) + "/upload");
    http.addHeader("Content-Type",
                   String("multipart/form-data; boundary=") + boundary);

    int code = http.POST(body.data(), body.size());
    http.end();

    if (code != 201)
    {
        Serial.printf("[ERROR] uploadFile: POST returned %d for %s\n", code, sdPath);
        return false;
    }

    // 5. Move to /sent on success
    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE)
    {
        if (!SD.exists("/sent"))
            SD.mkdir("/sent");

        char destPath[64];
        snprintf(destPath, sizeof(destPath), "/sent/%s", basename);
        if (!SD.rename(sdPath, destPath))
            Serial.printf("[ERROR] uploadFile: rename %s -> %s failed\n", sdPath, destPath);

        xSemaphoreGive(sdMutex);
    }

    return true;
}