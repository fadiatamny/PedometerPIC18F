#include "stepsGraph.h"

BOOL CheckButtonPressed(void)
{
    if (PORTBbits.RB0 == 0)
        return TRUE;
    else
        return FALSE;
}

void clearScreen0(void)
{
    int i;
    for (i = 0; i < 8; i++)
    {
        oledPutROMString("                                 ", i, 0);
    }
}

void strepLine()
{
    int i, j;
    char line[3] = {0};
    sprintf(line, "- ");
    for (j = 2; j < 7;)
    {
        for (i = 20; i < 130;)
        {
            oledPutString(line, j, i, 1);
            i = i + 10;
        }
        j = j + 1;
    }
}

void verLine()
{
    int i = 0;
    char x0 = 20, y0 = 60, x1 = 20, y1 = 63;
    while (i < 60)
    {
        drawLine(x0, y0, x1, y1, thin);
        x0 = x0 + 10;
        x1 = x1 + 10;
        i = i + 5;
    }
}

void pedometerGraph(BYTE steps[100][1])
{
    int i, temp0, temp1;
    BYTE x0, y0, x1 = 20, y1;
    BYTE tempx, tempy;

    for (i = 0; i < 99; i++)
    {
        y1 = 40 + 15;
        y0 = 40 + 15;

        tempx = steps[i][0] / 25;
        tempy = (((float)steps[i][0] / 25) - tempx) * 10;
        tempx *= 10;
        tempx = tempx + tempy;
        y1 -= tempx;
        tempx = steps[i + 1][0] / 25;
        tempy = (((float)steps[i + 1][0] / 25) - tempx) * 10;
        tempx *= 10;
        tempx = tempx + tempy;
        y0 -= tempx;
        x0 = x1;
        x1 = x1 + 1;
        drawLine(x0, y0, x1, y1, thin);
    }
}

void graphBase(BYTE steps[100][1])
{
    BYTE x0 = 0;
    BYTE y0 = 59;
    BYTE x1 = 128;
    BYTE y1 = 59;
    char num[4] = {0};
    clearScreen0();
    sprintf(num, "100");
    oledPutString(num, 2, 0, 1);
    sprintf(num, "60");
    oledPutString(num, 4, 0, 1);
    sprintf(num, "30");
    oledPutString(num, 6, 0, 1);
    drawLine(x0, y0, x1, y1, thin);
    verLine();
    strepLine();
    pedometerGraph(steps);

    while (1)
    {
        if (CheckButtonPressed())
            break;
    }
}