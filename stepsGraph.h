#ifndef STEPSGRAF_H
#define STEPSGRAF_H

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "mtouch.h"
#include "BMA150.h"
#include "OledGraphics.h"
#include "my_oled.h"

void strepLine();
void graphBase(BYTE steps[100][1]);
BOOL CheckButtonPressed(void);
void clearScreen0(void);

#endif // STEPSGARF_H