#ifndef MY_OLED_H
#define MY_OLED_H

#include "oled.h"

void bigOledPutString(unsigned char *ptr, unsigned char page, unsigned char col, unsigned char);
void bigOledWriteChar1x(char letter, unsigned char page, unsigned char column, unsigned char);
void bigOledWriteCharRaw(BYTE seg);

#endif // MY_OLED_H