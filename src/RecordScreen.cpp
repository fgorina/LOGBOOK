#include "RecordScreen.h"
#include <Arduino.h>
#include <M5Tough.h>


RecordScreen::RecordScreen(int width, int height, const char *title, tState *state, time_t period) : Screen(width, height, title)
{
    this->state = state;
    this->period = period;
}
void RecordScreen::enter()
{
    Serial.println("RecordScreen::enter");  
    const ButtonColors off_clrs = {BLACK, CYAN, WHITE};
    const ButtonColors on_clrs = {BLUE, CYAN, WHITE};
    brecord = new Button(width / 2 + width / 8, height / 2 - 30, width / 4, 60, false, "Start", off_clrs, on_clrs, MC_DATUM);
    old_second_millis = millis();
    draw();
}

void RecordScreen::exit()
{
    Serial.println("RecordScreen::exit");

    if (brecord != nullptr)
    {
        brecord->delHandlers();
        brecord->hide(BLACK);
        delete (brecord);
        brecord = nullptr;
        Serial.println("Deleted brecord");
    }
}

void RecordScreen::draw()
{

    Serial.println("RecordScreen::draw");
    M5.Lcd.clear();

    brecord->draw();
    draw_distance();
    draw_data();
}

void RecordScreen::draw_distance()
{
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.fillRect(0, height - 80, width, 80, BLACK);
    M5.Lcd.fillRect(0, 40, width/2, height, BLACK);
    M5.Lcd.setTextDatum(BL_DATUM);
    duration(buffer, MAXBUFFER);
    M5.Lcd.drawString(buffer, 10, height - 10);
    M5.Lcd.setTextDatum(BR_DATUM);
    distance(buffer, MAXBUFFER);
    M5.Lcd.drawString(buffer, width - 10, height - 10);
    // Now Show COG i SOG

    M5.Lcd.setTextDatum(ML_DATUM);
    snprintf(buffer, MAXBUFFER, "COG %03.0f º", state->cog.heading/PI*180.0);
    M5.Lcd.drawString(buffer, 19, 75);

    snprintf(buffer, MAXBUFFER, "SOG %03.1f kt", state->sog.value * 3600.0 / 1852.0);
    M5.Lcd.drawString(buffer, 19, 100);

    snprintf(buffer, MAXBUFFER, u8"AWA %03.0f º", state->wind.angle / PI * 180.0);
    M5.Lcd.drawString(buffer, 19, 130);

    snprintf(buffer, MAXBUFFER, "AWS %03.1f kt", state->wind.speed * 3600.0 / 1852.0);
    M5.Lcd.drawString(buffer, 19, 155);
}

void RecordScreen::draw_data()
{
    updatedDateTime();
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.fillRect(0, 0, width, 40, BLACK);
    
    M5.Lcd.drawString(buffer, width / 2, 10);

    

}

int RecordScreen::run()
{
    
    if (millis() - old_second_millis >= 1000){
        old_second_millis = millis();
        draw_data();
    }
    if (recording)
    {
        
        if (millis() - old_millis > period)
        {
            Serial.print("*");
            old_millis = millis();
            double lat2 = state->position.latitude;
            double lon2 = state->position.longitude;
            saveData(file);

            // Update distance only if distance > 10m. Perhaps should be better
            // doing 20m. Depends GPS accuracy

            double distance = haversine(old_lat, old_lon, lat2, lon2);
            if (abs(distance) > (10.0 / 1982.0))
            {
                miles += distance;
                old_lat = lat2;
                old_lon = lon2;
                draw_distance();
            }else{
                Serial.println("");
                Serial.print(lat2); Serial.print(" "); Serial.print(lon2); Serial.print(" ");
                Serial.println(distance);
            }
        }
    }
    if (brecord != nullptr && brecord->wasReleased())
    {
        if(!recording){
            startRecord();
            return -1;
        }else{
            stopRecord();
            return 0;
        }
        
        return (0);
    }
    return -1;
}

void RecordScreen::startRecord()
{

    // Get a file name and open it
    // Store first record
    // Start timer to record every xxx seconds
    const ButtonColors selected_clrs = {RED, WHITE, WHITE};

    Serial.println("Starting recording");
    brecord->setLabel("Stop");
    brecord->off = selected_clrs;
    draw();
    recording = true;
    start_millis = millis();
    start_time = time(nullptr);
    miles = 0.0;


    // We build a new filename
    newFilename(filename, LENNAME);
    file = SD.open(filename, FILE_WRITE);
    saveHeader(file, filename);
    saveData(file);
    old_millis = millis();

    old_lat = state->position.latitude;
    old_lon = state->position.longitude;
}

void RecordScreen::stopRecord()
{

    // Stop timer
    // Write data to file
    // Close file
    brecord->setLabel("Start");
    saveData(file);
    saveFooter(file);
    file.close();
    file = File();
    recording = false;
}

void RecordScreen::updatedDateTime()
{

    struct tm timeinfo;
   
    if (getLocalTime(&timeinfo))
    {
        strftime(buffer, 64, "%d-%m-%y %H:%M:%S", (const tm *)&timeinfo);
    }
    else
    {
        Serial.println("Problemes amb RTC");
        time_t now = time(nullptr);
        struct tm *ti = localtime(&now);
        strftime(buffer, 64, "%y-%m-%d %H:%M:%S", ti);
    }
}
void RecordScreen::newFilename(char *buff, int maxbuff)
{
    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    if(xmlFormat){
        strftime(buff, maxbuff, "/%y%m%d_%H%M.gpx", timeinfo);
    }else{
        strftime(buff, maxbuff, "/%y%m%d_%H%M.csv", timeinfo);
    }
    
}

void RecordScreen::duration(char *buff, int bufsize)
{
    long delta = millis() - start_millis;
    int minutes = floor(delta / 60000);
    int hours = floor(minutes / 60);
    int mins = minutes - (hours * 60);
    snprintf(buff, bufsize, "T %02d:%02d", hours, mins);
}

void RecordScreen::distance(char *buff, int bufsize)
{
    snprintf(buff, bufsize, "%.2f nmi", miles);
}

/*
Computes the distance between 2 points.

lat1, lon1, lat2, lon2 are in degrees
Result is in Nauctical miles
*/
double RecordScreen::haversine(double lat1, double lon1, double lat2, double lon2)
{

    double R = 3443.92;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double c = 2 * asin(sqrt(a));
    return R * c;
}

void  RecordScreen::saveHeader(File f, char* name){
    if(xmlFormat){
        state->saveGPXHeader(f, name);
    }else{
        file.println("Time\tLongitut\tLatitut\tCog\tSog\tHeading\tPitch\tRoll\tRot\tAwa\tAws");
    }
}

void  RecordScreen::saveFooter(File f){

     if(xmlFormat){
        state->saveGPXFooter(f);
    }
}

void RecordScreen::saveData(File f){
    if(xmlFormat){
        state->saveGPXTrackpoint(f);
    }else{
        state->saveCsv(f);
    }
}