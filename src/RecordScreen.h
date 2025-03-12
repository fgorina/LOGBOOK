#ifndef _RecordScreen_H_
#define _RecordScreen_H_


#include "Screen.h"
#include "State.h"
#include <SD.h>

class RecordScreen : public Screen
{   
    public:
    RecordScreen(int width, int height, const char* title, tState* state, time_t period); // period of writing in millis
    void enter();
    void exit();
    void draw();
    int run();

    protected: 
    
    static const int LENNAME = 31;
    static const int MAXBUFFER = 64;
    bool xmlFormat = false;
    Button *brecord = nullptr;
    tState* state;
    char buffer[64];
    bool recording = false;

    time_t period;
    time_t old_millis = 0;  // When we did last write
    time_t old_second_millis = 0;  // When we did last write

    time_t start_millis = 0; // millis at start of recprding
    time_t start_time = 0;  // Time at start of recording

    double old_lat = 0;
    double old_lon = 0;

    double miles = 0.0;

    File file = File();
    char filename[LENNAME-1] = "";



    void startRecord();
    void stopRecord();
    void updatedDateTime();
    void draw_distance();
    void draw_data();
    void newFilename(char* buff, int maxbuff);
    void duration(char* buff, int bufsize);   
    void distance(char* buff, int bufsize);
    void saveData(File f);
    void  saveHeader(File f, char* name);
    void  saveFooter(File f);
    double haversine(double lat1, double lon1, double lat2, double lon2);   // Distances lox in Nm
    size_t compressFile( const String &inputFilename);
};

#endif