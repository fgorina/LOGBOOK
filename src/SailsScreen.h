#pragma once

#include "Screen.h"
#include "State.h"

/* Sails Screen

    Lists the sails on board (state->sailsAvailable) with a button per state
    at the right of each one: F (full), 1, 2, 3 (reefs) and L (lowered).
    Touching one sets the sail state logged from that moment on.

    It is shown by RecordScreen on top of itself instead of through switchTo(),
    so the recording keeps running. run() returns 0 when the user is done.
*/

class SailsScreen : public Screen
{
public:
    SailsScreen(int width, int height, const char *title, tState *state);
    void enter() override;
    void exit() override;
    void draw() override;
    int run(const m5::touch_detail_t &t) override;

protected:
    static const int ROWS = 5;   // Sails per page
    static const int STATES = 5; // F, 1, 2, 3, L
    static const int TOP = 36;   // Y of first row
    static const int ROW_H = 40;
    static const int NAME_W = 110;
    static const int STATE_W = 42;

    tState *state;
    Button *bdone = nullptr;
    Button *bpage = nullptr;
    Button *bstate[ROWS][STATES] = {};
    int rowSail[ROWS] = {-1, -1, -1, -1, -1}; // Sail index shown in each row, -1 if empty
    int page = 0;
    int nPages = 1;

    void createRows();
    void deleteRows();
    void drawRow(int r);
    void deleteButton(Button *&b);
};
