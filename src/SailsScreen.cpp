#include "SailsScreen.h"

void writeSailState(); // main.cpp, persists state->sails

static const char *const STATE_LABELS[] = {"F", "1", "2", "3", "L"};
static const int STATE_VALUES[] = {1, 2, 3, 4, 0};

static const ButtonColors off_clrs = {BLACK, CYAN, WHITE};
static const ButtonColors on_clrs = {BLUE, CYAN, WHITE};
static const ButtonColors selected_clrs = {RED, WHITE, WHITE};

SailsScreen::SailsScreen(int width, int height, const char *title, tState *state) : Screen(width, height, title)
{
    this->state = state;
}

void SailsScreen::enter()
{
    Serial.println("SailsScreen::enter");

    int n = 0;
    for (int i = 0; i < tState::N_SAILS; i++)
    {
        if (state->hasSail(i))
            n++;
    }
    nPages = n > 0 ? (n + ROWS - 1) / ROWS : 1;
    page = 0;

    bdone = new Button(width - 80, 2, 78, 30, false, "OK", off_clrs, on_clrs, MC_DATUM);
    if (nPages > 1)
        bpage = new Button(width - 160, 2, 74, 30, false, "More", off_clrs, on_clrs, MC_DATUM);

    createRows();
    draw();
}

void SailsScreen::exit()
{
    Serial.println("SailsScreen::exit");
    deleteRows();
    deleteButton(bdone);
    deleteButton(bpage);
}

void SailsScreen::draw()
{
    M5.Display.clear();
    M5.Display.setFont(&fonts::FreeSans9pt7b);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextDatum(ML_DATUM);
    M5.Display.drawString(title, 4, 17);

    if (bdone != nullptr)
        bdone->draw();
    if (bpage != nullptr)
        bpage->draw();

    if (rowSail[0] < 0)
    {
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setTextDatum(MC_DATUM);
        M5.Display.drawString("No sails defined", width / 2, height / 2 - 12);
        M5.Display.drawString("Set them in Preferences", width / 2, height / 2 + 12);
        return;
    }

    for (int r = 0; r < ROWS; r++)
        drawRow(r);
}

int SailsScreen::run(const m5::touch_detail_t &t)
{
    if (state->displaySaver != DISPLAY_ACTIVE)
        return -1;

    if (state->sailsDirty)
    {
        state->sailsDirty = false;
        for (int r = 0; r < ROWS; r++)
            drawRow(r);
    }

    if (bdone != nullptr && bdone->handleTouch(t))
        return 0;

    if (bpage != nullptr && bpage->handleTouch(t))
    {
        deleteRows();
        page = (page + 1) % nPages;
        createRows();
        draw();
        return -1;
    }

    for (int r = 0; r < ROWS; r++)
    {
        for (int s = 0; s < STATES; s++)
        {
            if (bstate[r][s] != nullptr && bstate[r][s]->handleTouch(t))
            {
                state->sails[rowSail[r]] = STATE_VALUES[s];
                writeSailState();
                drawRow(r);
                return -1;
            }
        }
    }
    return -1;
}

// Creates the state buttons for the sails in the current page

void SailsScreen::createRows()
{
    int skip = page * ROWS;
    int r = 0;

    for (int i = 0; i < ROWS; i++)
        rowSail[i] = -1;

    for (int i = 0; i < tState::N_SAILS && r < ROWS; i++)
    {
        if (!state->hasSail(i))
            continue;
        if (skip > 0)
        {
            skip--;
            continue;
        }
        rowSail[r] = i;
        int y = TOP + r * ROW_H;
        for (int s = 0; s < STATES; s++)
        {
            bstate[r][s] = new Button(NAME_W + s * STATE_W, y + 3, STATE_W - 4, ROW_H - 6, false,
                                      STATE_LABELS[s], off_clrs, on_clrs, MC_DATUM);
        }
        r++;
    }
}

void SailsScreen::deleteRows()
{
    for (int r = 0; r < ROWS; r++)
    {
        for (int s = 0; s < STATES; s++)
            deleteButton(bstate[r][s]);
    }
}

// Draws sail name and state buttons, the active state highlighted

void SailsScreen::drawRow(int r)
{
    int sail = rowSail[r];
    if (sail < 0)
        return;

    int y = TOP + r * ROW_H;
    M5.Display.setFont(&fonts::FreeSans9pt7b);
    M5.Display.fillRect(0, y, NAME_W, ROW_H, BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextDatum(ML_DATUM);
    M5.Display.drawString(tState::SAIL_NAMES[sail], 4, y + ROW_H / 2);

    for (int s = 0; s < STATES; s++)
    {
        Button *b = bstate[r][s];
        b->off = (state->sails[sail] == STATE_VALUES[s]) ? selected_clrs : off_clrs;
        b->draw();
    }
}

void SailsScreen::deleteButton(Button *&b)
{
    if (b != nullptr)
    {
        b->delHandlers();
        b->hide(BLACK);
        delete (b);
        b = nullptr;
    }
}
