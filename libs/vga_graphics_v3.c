#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
// Our assembled programs:
// Each gets the name <pio_filename.pio.h>
#include "hsync.pio.h"
#include "vsync.pio.h"
#include "rgb.pio.h"
// Header file
#include "vga16_graphics_v2.h"
// Font file
#include <string.h>
#include "vga_graphics_v3.h"

#define DISP_WIDTH 630
#define DISP_HEIGHT 475

#define TOP_MARGIN 10
#define BTM_MARGIN 10
#define LFT_MARGIN 10
#define RHT_MARGIN 10

#define G_TOP_MARGIN 5
#define G_BTM_MARGIN 5
#define G_LFT_MARGIN 5
#define G_RHT_MARGIN 5

int columns;
int rows;

int eff_w = DISP_WIDTH - LFT_MARGIN - RHT_MARGIN;
int eff_h = DISP_HEIGHT - TOP_MARGIN - BTM_MARGIN;

char screentext[40];

typedef struct {
    int x;
    int y;

    bool dbg;
} grid_element;

typedef struct {
    int x;
    int y;

    bool dbg;
    char *text;
} textbox;

typedef struct {
    int x0;
    int y0;
} gcoord;

// Globals
bool dbg_flag = true;
float x_sep;
float y_sep;

gcoord grid_coordinates;

void gridInit(int c, int r) {
    if (dbg_flag) {
        drawRect(0, 0, DISP_WIDTH, DISP_HEIGHT, GREEN);
    }

    grid_element grid[c][r];

    columns = c;
    rows = r;

    x_sep = (float)(DISP_WIDTH - LFT_MARGIN - RHT_MARGIN) / c;
    y_sep = (float)(DISP_HEIGHT - TOP_MARGIN - BTM_MARGIN) / r;

    for (int i = 0; i < c; i++) {
        for (int j = 0; j < r; j++) {
            grid[i][j].dbg = dbg_flag;

            grid[i][j].x = (int)(LFT_MARGIN + i * x_sep);
            grid[i][j].y = (int)(TOP_MARGIN + j * y_sep);
            
            // Draw grid outline if debug flag is true
            if (grid[i][j].dbg) {
                drawRect(grid[i][j].x, grid[i][j].y, x_sep, y_sep, RED);
                
            }
        }
    }

    
}

void gSetCursor(int gridCol, int gridRow) {
    grid_coordinates.x0 = LFT_MARGIN + gCol2X(gridCol);
    grid_coordinates.y0 = TOP_MARGIN + gRow2Y(gridRow);
}

int g2ScreenX(int gx) { return grid_coordinates.x0 + G_LFT_MARGIN + gx; }
int g2ScreenY(int gy) { return grid_coordinates.y0 + G_TOP_MARGIN + gy; }

int gCol2X(int gCol) { return (int)(gCol * ((float)eff_w/(columns))); }

int gRow2Y(int gRow) { return (int)(gRow * ((float)eff_h/(rows))); }

void gDrawTextbox(int gx, int gy, char* content, bool snap) {
    setTextSize(1);
    setTextColor(WHITE);

    int text_width = strlen(content) * 7 + 5;

    if (snap) {
        drawRoundRect(g2ScreenX(gx), g2ScreenY(gy), text_width, 15, 5, CYAN);
        setCursor(g2ScreenX(gx + 5), g2ScreenY(gy + 4));

        sprintf(screentext, content);
        writeString(screentext);
    }
}

