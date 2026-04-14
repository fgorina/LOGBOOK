#ifndef _RecordScreen_H_
#define _RecordScreen_H_


#include "Screen.h"
#include "State.h"
#include <SD.h>

class RecordScreen : public Screen
{   
    public:
    bool xmlFormat = false;
    RecordScreen(int width, int height, const char* title, tState* state, time_t period); // period of writing in millis
    void enter() override;
    void exit() override;
    void draw() override;
    int run(const m5::touch_detail_t &t) override;

    protected: 
    
    static const int LENNAME = 31;
    static const int MAXBUFFER = 64;
    
    Button *brecord = nullptr;
    tState* state;
    char buffer[64];
    bool recording = false;

    time_t period;
    time_t old_millis = 0;  // When we did last write
    time_t old_second_millis = 0;  // When we did last write

    time_t start_millis = 0; // millis at start of recprding
    time_t start_time = 0;  // Time at start of recording

    int rows_not_saved = 0;
    int max_rows_not_saved = 60;    // Aroun 1'

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

    // Moving filter (EMA on velocity vector)
    float fastX_ = 0, fastY_ = 0;
    float slowX_ = 0, slowY_ = 0;
    bool  filterMoving_      = false;
    bool  filterInitialized_ = false;

    static constexpr float START_THRESH = 0.26f;   // m/s fast EMA → start
    static constexpr float STOP_THRESH  = 0.15f;   // m/s slow EMA → stop
    static constexpr float ALPHA_FAST   = 0.0056f;
    static constexpr float ALPHA_SLOW   = 0.0017f;

    void updateMovingFilter();
};

#endif