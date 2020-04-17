
#include "my_oled.h"

ROM BYTE g_bigFont[4][1] = {
    {0x00},
    {0x0f},
    {0xf0},
    {0xff},
};

void bigOledWriteCharRaw(BYTE seg)
{
    int i = 0;
    BYTE a[4] = {0x0, 0x0, 0x0, 0x0};

    a[3] = 0b11000000 & seg;
    a[2] = 0b00110000 & seg;
    a[1] = 0b00001100 & seg;
    a[0] = 0b00000011 & seg;

    a[3] = a[3] >> 6;
    a[2] = a[2] >> 4;
    a[1] = a[1] >> 2;

    for (i = 0; i < 4; ++i)
    {
        WriteData(0xff);
        WriteData(0x00);
        WriteData(0xff);
    }
}

void myWriteData(BYTE seg, unsigned char page, unsigned char col, unsigned char flag)
{
    int i = 0;
    BYTE a[4] = {0x0, 0x0, 0x0, 0x0};

    a[3] = 0b11000000 & seg;
    a[2] = 0b00110000 & seg;
    a[1] = 0b00001100 & seg;
    a[0] = 0b00000011 & seg;

    a[3] = a[3] >> 6;
    a[2] = a[2] >> 4;
    a[1] = a[1] >> 2;

    if (flag == 0)
    {
        WriteCommand(page);
        WriteCommand(0x00 + (col & 0x0F));
        WriteCommand(0x10 + ((col >> 4) & 0x0F));
        WriteData(g_bigFont[a[0]][0]);
        WriteCommand(page + 1);
        WriteCommand(0x00 + (col & 0x0F));
        WriteCommand(0x10 + ((col >> 4) & 0x0F));
        WriteData(g_bigFont[a[1]][0]);
        WriteCommand(page + 2);
        WriteCommand(0x00 + (col & 0x0F));
        WriteCommand(0x10 + ((col >> 4) & 0x0F));
        WriteData(g_bigFont[a[2]][0]);
        WriteCommand(page + 3);
        WriteCommand(0x00 + (col & 0x0F));
        WriteCommand(0x10 + ((col >> 4) & 0x0F));
        WriteData(g_bigFont[a[3]][0]);
    }
    else
    {
        WriteCommand(page);
        WriteCommand(0x00 + (col & 0x0F));
        WriteCommand(0x10 + ((col >> 4) & 0x0F));
        WriteData(~g_bigFont[a[0]][0]);
        WriteCommand(page + 1);
        WriteCommand(0x00 + (col & 0x0F));
        WriteCommand(0x10 + ((col >> 4) & 0x0F));
        WriteData(~g_bigFont[a[1]][0]);
        WriteCommand(page + 2);
        WriteCommand(0x00 + (col & 0x0F));
        WriteCommand(0x10 + ((col >> 4) & 0x0F));
        WriteData(~g_bigFont[a[2]][0]);
        WriteCommand(page + 3);
        WriteCommand(0x00 + (col & 0x0F));
        WriteCommand(0x10 + ((col >> 4) & 0x0F));
        WriteData(~g_bigFont[a[3]][0]);
    }
}

void myOledWriteCharRaw(char letter, unsigned char page, unsigned char col, unsigned char flag)
{
    letter -= ' ';                                      // Adjust character to table that starts at 0x20
    myWriteData(g_pucFont[letter][0], page, col, flag); // Write first column
    col = col + 1;
    myWriteData(g_pucFont[letter][1], page, col, flag); // Write second column
    col = col + 1;
    myWriteData(g_pucFont[letter][2], page, col, flag); // Write third column
    col = col + 1;
    myWriteData(g_pucFont[letter][3], page, col, flag); // Write fourth column
    col = col + 1;
    myWriteData(g_pucFont[letter][4], page, col, flag);
    col = col + 1;                      // Write fifth column
    myWriteData(0x00, page, col, flag); // Write 1 column for buffer to next character
    return;
}

void myOledWriteChar1x(char letter, unsigned char page, unsigned char column, unsigned char flag)
{

    WriteCommand(page);
    column += OFFSET;
    WriteCommand(0x00 + (column & 0x0F));
    WriteCommand(0x10 + ((column >> 4) & 0x0F));
    myOledWriteCharRaw(letter, page, column, flag);
    return;
}

void bigOledPutString(unsigned char *ptr, unsigned char page, unsigned char col, unsigned char flag)
{
    page = page + 0xB0;
    myOledWriteChar1x(*ptr, page, col, flag);

    while (*++ptr)
    {
        col = col + 9;
        myOledWriteCharRaw(*ptr, page, col, flag);
    }
}