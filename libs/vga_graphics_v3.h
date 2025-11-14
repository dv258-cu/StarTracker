void gridInit(int c, int r);
int gCol2X(int gCol);
int gRow2Y(int gRow);
void gSetCursor(int gridCol, int gridRow);

int g2ScreenX(int gx);
int g2ScreenY(int gy);

void gDrawTextbox(int gx, int gy, char* content, bool snap);