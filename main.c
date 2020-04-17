/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:   PIC18 or PIC24 USB Microcontrollers
 Hardware:    The code is natively intended to be used on the following
        hardware platforms: PICDEM� FS USB Demo Board, 
        PIC18F87J50 FS USB Plug-In Module, or
        Explorer 16 + PIC24 USB PIM.  The firmware may be
        modified for use on other USB platforms by editing the
        HardwareProfile.h file.
 Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:   Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/

//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "stepsGraph.h"
#include "soft_start.h"

//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
//Watchdog Timer Enable bit:
#pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit) \
                                    //PLL Prescaler Selection bits:
#pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input) \
                                    //Stack Overflow/Underflow Reset Enable bit:
#pragma config STVREN = ON          //Reset on stack overflow/underflow enabled \
                                    //Extended Instruction Set Enable bit:
#pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode) \
                                    //CPU System Clock Postscaler:
#pragma config CPUDIV = OSC1        //No CPU system clock divide \
                                    //Code Protection bit:
#pragma config CP0 = OFF            //Program memory is not code-protected \
                                    //Oscillator Selection bits:
#pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB \
                                    //Secondary Clock Source T1OSCEN Enforcement:
#pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected \
                                    //Low-Power Timer1 Oscillator Enable bit:
#pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation \
                                    //Fail-Safe Clock Monitor Enable bit:
#pragma config FCMEN = OFF          //Fail-Safe Clock Monitor disabled \
                                    //Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
#pragma config IESO = OFF           //Two-Speed Start-up disabled \
                                    //Watchdog Timer Postscaler Select bits:
#pragma config WDTPS = 32768        //1:32768 \
                                    //DSWDT Reference Clock Select bit:
#pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock \
                                    //RTCC Reference Clock Select bit:
#pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock \
                                    //Deep Sleep BOR Enable bit:
#pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes) \
                                    //Deep Sleep Watchdog Timer Enable bit:
#pragma config DSWDTEN = OFF        //Disabled \
                                    //Deep Sleep Watchdog Timer Postscale Select bits:
#pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds) \
                                    //IOLOCK One-Way Set Enable bit:
#pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed \
                                    //MSSP address mask:
#pragma config MSSP7B_EN = MSK7     //7 Bit address masking \
                                    //Write Protect Program Flash Pages:
#pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0 \
                                    //Write Protection End Page (valid when WPDIS = 0):
#pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0] \
                                    //Write/Erase Protect Last Page In User Flash bit:
#pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled \
                                    //Write Protect Disable bit:
#pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored

#else
#error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif

//	========================	Global VARIABLES	========================
#pragma udata
unsigned char millSec = 0;
unsigned char day = 1;
unsigned char month = 1;
unsigned char hour = 12;
unsigned char minute = 30;
unsigned char second = 0;
unsigned int year = 2000;
unsigned char toprint[10] = {0};
BYTE steps[100][1] = {0};
BYTE currSteps = 0;
unsigned char stepsInputs[3] = {0};
unsigned char stepsInputsF[2] = {0};
unsigned char ampmMode = 1;
unsigned char analogMode = 0;
unsigned char x1h = 0;
unsigned char y1h = 0;
unsigned char x1m = 0;
unsigned char y1m = 0;
unsigned char RA0 = '0', RA1 = '1', RA2 = '2', RA3 = '3';
BOOL initClock = 1;
char clock[60][2] = {
    {67, 0}, {71, 1}, {74, 2}, {79, 3}, {81, 4}, {82, 5}, {84, 6}, {88, 9}, {91, 12}, {92, 14}, {93, 16}, {95, 18}, {96, 21}, {97, 25}, {98, 29}, {98, 32}, {98, 35}, {97, 39}, {96, 43}, {95, 45}, {93, 47}, {92, 50}, {89, 53}, {85, 56}, {84, 58}, {82, 58}, {79, 60}, {76, 61}, {73, 62}, {70, 63}, {67, 63}, {64, 63}, {60, 62}, {57, 61}, {54, 60}, {51, 58}, {47, 56}, {45, 54}, {43, 52}, {41, 50}, {40, 47}, {38, 44}, {37, 41}, {36, 38}, {35, 35}, {35, 32}, {35, 29}, {36, 26}, {37, 22}, {38, 18}, {40, 16}, {41, 13}, {44, 10}, {47, 7}, {49, 6}, {51, 5}, {52, 4}, {54, 3}, {57, 2}, {61, 1}};

//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode(void);
static void YourLowPriorityISRCode(void);
void updateClockTime(void);
void shiftStepsArray(void);
void detectStep(void);
void mainTraverse(int);
void mainMenu(void);
void bigClockDisplay(void);
void bigDigitalClockDisplay(void);
void displayToggle(void);
int CheckUDVolt(unsigned int, unsigned int);
int CheckLRVolt(unsigned int);
int GetAccVal(char);
void ProtectedBigOledPutString(rom unsigned char *, unsigned char, unsigned char, unsigned char);
void ProtectedOledPutString(rom unsigned char *, unsigned char, unsigned char, unsigned char);
static void _DelayMs(WORD);
void checkMonth(void);
int checkLeap(unsigned int);
void angClock(void);
void clockFrame(void);
void amPmToggle(void);
void setTime(void);
void setDate(void);
void pedometerView(void);
void smallClock(void);
void smallDate(void);
void smallAMPM(void);
void bigClock(void);
void analogClock(void);
void icon(void);
void showSteps(void);

//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
//the reset, high priority interrupt, and low priority interrupt
//vectors.  However, the current Microchip USB bootloader
//examples are intended to occupy addresses 0x00-0x7FF or
//0x00-0xFFF depending on which bootloader is used.  Therefore,
//the bootloader code remaps these vectors to new locations
//as indicated below.  This remapping is only necessary if you
//wish to program the hex file generated from this project with
//the USB bootloader.  If no bootloader is used, edit the
//usb_config.h file and comment out the following defines:
//#define PROGRAMMABLE_WITH_SD_BOOTLOADER

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
#define REMAPPED_RESET_VECTOR_ADDRESS 0xA000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS 0xA008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
#else
#define REMAPPED_RESET_VECTOR_ADDRESS 0x00
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS 0x08
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
#endif

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
extern void _startup(void); // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset(void)
{
  _asm goto _startup _endasm
}
#endif
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void Remapped_High_ISR(void)
{
  _asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void Remapped_Low_ISR(void)
{
  _asm goto YourLowPriorityISRCode _endasm
}

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
//Note: If this project is built while one of the bootloaders has
//been defined, but then the output hex file is not programmed with
//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
//As a result, if an actual interrupt was enabled and occured, the PC would jump
//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
//would effective reset the application.

//To fix this situation, we should always deliberately place a
//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
//hex file of this project is programmed with the bootloader, these sections do not
//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
//programmed using the bootloader, then the below goto instructions do get programmed,
//and the hex file still works like normal.  The below section is only required to fix this
//scenario.
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR(void)
{
  _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR(void)
{
  _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#endif //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"

#pragma code

//	========================	Application Interrupt Service Routines	========================

void hexout(BYTE cnt)
{
  if (cnt < 10)
    oledWriteCharRaw('0' + cnt);
  else
    oledWriteCharRaw('A' + cnt - 10);
}

//These are your actual interrupt handling routines.
#pragma interrupt YourHighPriorityISRCode
static void YourHighPriorityISRCode()
{
  // static int count = 0;
  //Check which interrupt flag caused the interrupt.
  //Service the interrupt
  //Clear the interrupt flag
  //Etc.
  // ++count;
  updateClockTime(); // deals with the time updating
  // oledPutROMString("Count:", 2, 12);
  // hexout((count >> 4) & 0xF);
  // hexout(count & 0xF);
  INTCONbits.TMR0IF = 0;

} //This return will be a "retfie fast", since this is in a #pragma interrupt section
#pragma interruptlow YourLowPriorityISRCode
static void YourLowPriorityISRCode()
{
  //Check which interrupt flag caused the interrupt.
  //Service the interrupt
  //Clear the interrupt flag
  //Etc.

} //This return will be a "retfie", since this is in a #pragma interruptlow section
#endif

//	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char *

/******************************************************************************
 * Function:        static void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the application code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
static void UserInit(void)
{

  // Soft Start the APP_VDD
  while (!AppPowerReady())
    ;

  /* Initialize the mTouch library */
  mTouchInit();

  /* Call the mTouch callibration function */
  mTouchCalibrate();

  /* Initialize the accelerometer */
  InitBma150();

  /* Initialize the oLED Display */
  ResetDevice();
  FillDisplay(0x00);

} //end UserInit

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
#if (defined(__18CXX) & !defined(PIC18F87J50_PIM))
  ADCON1 |= 0x0F; // Default all pins to digital

#if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
  //On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
  //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
  //This allows the device to power up at a lower initial operating frequency, which can be
  //advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
  //operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
  //power up the PLL.
  {
    unsigned int pll_startup_counter = 600;
    OSCTUNEbits.PLLEN = 1; //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
    while (pll_startup_counter--)
      ;
  }
//Device switches over automatically to PLL output after PLL is locked and ready.
#endif

#if defined(PIC18F46J50_PIM)
  //Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
  //use the ANCONx registers to control this, which is different from other devices which
  //use the ADCON1 register for this purpose.
  ANCON0 = 0xFF; // Default all pins to digital
  ANCON1 = 0xFF; // Default all pins to digital
#endif

  //  The USB specifications require that USB peripheral devices must never source
  //  current onto the Vbus pin.  Additionally, USB peripherals should not source
  //  current on D+ or D- when the host/hub is not actively powering the Vbus line.
  //  When designing a self powered (as opposed to bus powered) USB peripheral
  //  device, the firmware should make sure not to turn on the USB module and D+
  //  or D- pull up resistor unless Vbus is actively powered.  Therefore, the
  //  firmware needs some means to detect when Vbus is being powered by the host.
  //  A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
  //  can be used to detect when Vbus is high (host actively powering), or low
  //  (host is shut down or otherwise not supplying power).  The USB firmware
  //  can then periodically poll this I/O pin to know when it is okay to turn on
  //  the USB module/D+/D- pull up resistor.  When designing a purely bus powered
  //  peripheral device, it is not possible to source current on D+ or D- when the
  //  host is not actively providing power on Vbus. Therefore, implementing this
  //  bus sense feature is optional.  This firmware can be made to use this bus
  //  sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
  //  HardwareProfile.h file.
#if defined(USE_USB_BUS_SENSE_IO)
  tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

  //  If the host PC sends a GetStatus (device) request, the firmware must respond
  //  and let the host know if the USB peripheral device is currently bus powered
  //  or self powered.  See chapter 9 in the official USB specifications for details
  //  regarding this request.  If the peripheral device is capable of being both
  //  self and bus powered, it should not return a hard coded value for this request.
  //  Instead, firmware should check if it is currently self or bus powered, and
  //  respond accordingly.  If the hardware has been configured like demonstrated
  //  on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
  //  currently selected power source.  On the PICDEM FS USB Demo Board, "RA2"
  //  is used for this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
  //  has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
  //  to it in HardwareProfile.h.
#if defined(USE_SELF_POWER_SENSE_IO)
  tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

  UserInit();

} //end InitializeSystem

//	========================	Application Code	========================

int checkLeap(unsigned int year)
{
  if (year % 4 == 0)
  {
    if (year % 100 == 0)
    {
      if (year % 400 == 0)
        return 1;
      else
        return 0;
    }
    else
      return 1;
  }
  else
    return 0;
}

void checkMonth(void)
{
  switch (month)
  {
  case 1:
  case 3:
  case 5:
  case 7:
  case 8:
  case 10:
  case 12:
    if (day > 31)
    {
      day = 1;
      if (month == 12)
      {
        month = 1;
        ++year;
      }
    }
    break;
  default:
    if ((checkLeap(year) && month == 2 && day > 29) || (month == 2 && day > 28) || day > 30)
    {
      day = 1;
      ++month;
    }
  }
}

void updateClockTime(void)
{
  ++millSec;
  if (millSec >= 100)
  {
    millSec = 0;
    detectStep();
    ++second;
    if (second >= 60)
    {
      second = 0;
      shiftStepsArray();
      steps[119][0] = currSteps;
      currSteps = 0;
      ++minute;
      if (minute >= 60)
      {
        minute = 0;
        ++hour;
        if (hour >= 24)
        {
          hour = 0;
          ++day;
          checkMonth();
        }
      }
    }
  }
}

void shiftStepsArray(void)
{
  int i = 0;
  for (i = 0; i < 120 - 1; ++i)
    steps[i][0] = steps[i + 1][0];
}

int CheckUDVolt(unsigned int x, unsigned int y)
{
  if (x < y && y - x > 75 && x < 800)
  {
    return 1; //Up
  }
  else if (y < x && x - y > 200 && y < 900)
  {
    return 2;
  } //Down
  else
    return 0; //Not pushed
}

int CheckLRVolt(unsigned int x)
{
  if (x > 600)
    return 0;
  else
    return 1;
}

int GetAccVal(char c)
{
  BYTE msb, lsb;
  BYTE mask = 0b10000000;
  int signextend = 0xFC00;
  int val = 0, n1, n2;
  if (c == 'x')
  {
    n1 = 3;
    n2 = 2;
  }
  if (c == 'y')
  {
    n1 = 5;
    n2 = 4;
  }
  if (c == 'z')
  {
    n1 = 7;
    n2 = 6;
  }
  msb = BMA150_ReadByte(n1);
  lsb = BMA150_ReadByte(n2);
  lsb = lsb >> 6;
  val += (int)msb;
  val = val << 2;
  val += (int)lsb;
  mask = mask & msb;
  if (mask == 0b10000000)
  {
    val |= signextend;
  }
  return val;
}

void ProtectedOledPutString(rom unsigned char *ptr, unsigned char page, unsigned char col, unsigned char flag)
{
  INTCONbits.T0IE = 0; //Timer0 Overflow Interrupt Disable
  oledPutString(ptr, page, col, flag);
  INTCONbits.T0IE = 1;
}

void ProtectedBigOledPutString(rom unsigned char *ptr, unsigned char page, unsigned char col, unsigned char flag)
{
  INTCONbits.T0IE = 0; //Timer0 Overflow Interrupt Disable
  bigOledPutString(ptr, page, col, flag);
  INTCONbits.T0IE = 1; //Timer0 Overflow Interrupt Enabled
}

#define DELAY_1MS 32000 / 9
static void _DelayMs(WORD time)
{
  unsigned delay;
  while (time--)
    for (delay = 0; delay < DELAY_1MS; delay++)
      ;
}

void displayToggle(void)
{
  clearScreen0();
  while (1)
  {
    smallClock();
    if (CheckLRVolt(mTouchReadButton(RA0)))
    {
      if (++analogMode > 1)
        analogMode = 1;
      clearScreen0();
    }
    if (CheckLRVolt(mTouchReadButton(RA3)))
    {
      if (--analogMode < 0)
        analogMode = 0;
      clearScreen0();
    }

    if (CheckButtonPressed())
      break;

    sprintf(toprint, "Digital");
    ProtectedOledPutString(toprint, 9, 0, analogMode == 1 ? 1 : 0);
    sprintf(toprint, "Analog");
    ProtectedOledPutString(toprint, 9, 45, analogMode == 1 ? 0 : 1);
    DelayMs(50);
  }
}

void amPmToggle(void)
{
  clearScreen0();

  while (1)
  {
    smallClock();
    if (CheckLRVolt(mTouchReadButton(RA0)))
    {
      if (++ampmMode > 1)
        ampmMode = 1;
      clearScreen0();
    }
    if (CheckLRVolt(mTouchReadButton(RA3)))
    {
      if (--ampmMode < 0)
        ampmMode = 0;
      clearScreen0();
    }

    if (CheckButtonPressed())
      break;

    sprintf(toprint, "24H");
    ProtectedOledPutString(toprint, 9, 0, ampmMode == 1 ? 1 : 0);
    sprintf(toprint, "AM/PM");
    ProtectedOledPutString(toprint, 9, 45, ampmMode == 1 ? 0 : 1);
    DelayMs(50);
  }
}

void setTime(void)
{
  int index = 1;
  int h = hour;
  int m = minute;
  int s = second;

  clearScreen0();

  while (1)
  {
    smallClock();
    if (CheckLRVolt(mTouchReadButton(RA0)))
    {
      if (++index > 3)
        index = 3;
      clearScreen0();
    }
    if (CheckLRVolt(mTouchReadButton(RA3)))
    {
      if (--index < 0)
        index = 0;
      clearScreen0();
    }

    if (CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 1)
      switch (index)
      {
      case 1:
        if (++h > 23)
          h = 0;
        break;
      case 2:
        if (++m > 59)
          m = 0;
        break;
      case 3:
        if (++s > 59)
          s = 0;
        break;
      }

    if (CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 2)
      switch (index)
      {
      case 1:
        if (--h < 0)
          h = 23;
        break;
      case 2:
        if (--m < 0)
          m = 59;
        break;
      case 3:
        if (--s < 0)
          s = 59;
        break;
      }

    if (CheckButtonPressed())
      break;

    sprintf(toprint, "%d:", h);
    ProtectedBigOledPutString(toprint, 9, 0, index == 1 ? 1 : 0);

    sprintf(toprint, "%d:", m);
    ProtectedBigOledPutString(toprint, 9, h > 9 ? 20 : 15, index == 2 ? 1 : 0);

    sprintf(toprint, "%d", s);
    ProtectedBigOledPutString(toprint, 9, (h > 9 || m > 9) ? 40 : 30, index == 3 ? 1 : 0);
    DelayMs(50);
  }

  hour = h;
  minute = m;
  second = s;
}

void setDate(void)
{
  int index = 1;
  int y = year;
  int m = month;
  int d = day;

  while (1)
  {
    clearScreen0();
    smallClock();
    if (CheckLRVolt(mTouchReadButton(RA0)))
      if (++index > 3)
        index = 3;
    if (CheckLRVolt(mTouchReadButton(RA3)))
      if (--index < 1)
        index = 1;

    if (CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 1)
      switch (index)
      {
      case 1:
        ++y;
        break;
      case 2:
        if (++m > 12)
          m = 1;
        break;
      case 3:
        ++d;
        switch (m)
        {
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12:
          if (d > 31)
          {
            d = 1;
          }
          break;
        default:
          if ((checkLeap(y) && m == 2 && d > 29) || (m == 2 && d > 28) || d > 30)
          {
            d = 1;
          }
        }
        break;
      }

    if (CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 2)
      switch (index)
      {
      case 1:
        if (--y < 2000)
          y = 2000;
        break;
      case 2:
        if (--m < 1)
          m = 12;
        break;
      case 3:
        --d;
        switch (m)
        {
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12:
          if (d < 1)
          {
            d = 31;
          }
          break;
        default:
          if (checkLeap(y) && m == 2 && d < 1)
          {
            d = 29;
          }
          else if (m == 2 && d < 1)
          {
            d = 28;
          }
          else if (d < 30)
          {
            d = 30;
          }
        }
        break;
      }
    if (CheckButtonPressed())
      break;

    sprintf(toprint, "%d/", d);
    ProtectedBigOledPutString(toprint, 9, 0, index == 1 ? 1 : 0);

    sprintf(toprint, "%d/", m);
    ProtectedBigOledPutString(toprint, 9, d > 9 ? 20 : 15, index == 2 ? 1 : 0);

    sprintf(toprint, "%d", y);
    ProtectedBigOledPutString(toprint, 9, (d > 9 || m > 9) ? 40 : 30, index == 3 ? 1 : 0);
    DelayMs(50);
  }

  day = d;
  month = m;
  year = y;
}

void icon(void)
{

  int i = 0;
  if (currSteps > 0)
  {
    for (i = 0; i < 4; i++) //paint icon
    {
      drawLine(4, 2 + i, 8, 2 + i, thin);
    }

    for (i = 0; i < 4; i++)
    {
      drawLine(12, 2 + i, 16, 2 + i, thin);
    }

    for (i = 0; i < 4; i++)
    {
      drawLine(5, 4 + i, 7, 4 + i, thin);
    }

    for (i = 0; i < 4; i++)
    {
      drawLine(13, 4 + i, 15, 4 + i, thin);
    }
    DelayMs(50);
    for (i = 0; i < 4; i++) //paint icon
    {
      drawLine(4, 2 + i, 8, 2 + i, thin);
    }
    for (i = 0; i < 4; i++)
    {
      drawLine(12, 2 + i, 16, 2 + i, thin);
    }

    for (i = 0; i < 4; i++)
    {
      drawLine(5, 4 + i, 7, 4 + i, thin);
    }

    for (i = 0; i < 4; i++)
    {
      drawLine(13, 4 + i, 15, 4 + i, thin);
    }
  }
}

void pedometerView(void)
{
  graphBase(steps);
}

void detectStep(void)
{
  stepsInputs[0] = stepsInputs[1];
  stepsInputs[1] = stepsInputs[2];
  stepsInputs[2] = GetAccVal('z');

  stepsInputsF[0] = stepsInputsF[1];
  stepsInputsF[1] = GetAccVal('y');

  if (stepsInputs[2] > 100 || stepsInputs[2] < -100)
    if (stepsInputs[1] > stepsInputs[0] && stepsInputs[1] < stepsInputs[2])
    {
      if (stepsInputsF[1] > stepsInputsF[0] || stepsInputsF[1] < stepsInputsF[0])
        if (++currSteps > 100)
          currSteps = 100;
    }
}

void mainTraverse(int currChoice)
{
  switch (currChoice)
  {
  case 1:
    displayToggle();
    break;
  case 2:
    amPmToggle();
    break;
  case 3:
    setTime();
    break;
  case 4:
    setDate();
    break;
  case 5:
    pedometerView();
    break;
  }
}

void mainMenu(void)
{
  int currChoice = 1;
  clearScreen0();
  while (1)
  {
    smallClock();
    sprintf(toprint, "display");
    if (1 == currChoice)
      ProtectedOledPutString(toprint, 1, 2 * 6, 0);
    else
      ProtectedOledPutString(toprint, 1, 2 * 6, 1);

    sprintf(toprint, "12H/24H");
    if (2 == currChoice)
      ProtectedOledPutString(toprint, 2, 2 * 6, 0);
    else
      ProtectedOledPutString(toprint, 2, 2 * 6, 1);

    sprintf(toprint, "Set Time");
    if (3 == currChoice)
      ProtectedOledPutString(toprint, 3, 2 * 6, 0);
    else
      ProtectedOledPutString(toprint, 3, 2 * 6, 1);

    sprintf(toprint, "Set Date");
    if (4 == currChoice)
      ProtectedOledPutString(toprint, 4, 2 * 6, 0);
    else
      ProtectedOledPutString(toprint, 4, 2 * 6, 1);

    sprintf(toprint, "Pedometer");
    if (5 == currChoice)
      ProtectedOledPutString(toprint, 5, 2 * 6, 0);
    else
      ProtectedOledPutString(toprint, 5, 2 * 6, 1);

    if (CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 1) //Pressed up
      if (currChoice > 1)
      {
        currChoice--;
      }
    DelayMs(30);
    if (CheckUDVolt(mTouchReadButton(RA1), mTouchReadButton(RA2)) == 2) //Pressed down
      if (currChoice < 5)
      {
        currChoice++;
      }
    DelayMs(30);
    if (CheckLRVolt(mTouchReadButton(RA0)))
      mainTraverse(currChoice);

    if (CheckButtonPressed())
      break;
  }
}

void analogClock(void)
{
  char xhand, yhand, i;
  for (i = 0; i < 12; i++)
  {
    xhand = 67 + (9 * ((clock[i * 5][0] - 67) / 10));
    yhand = 32 + (9 * ((clock[i * 5][1] - 32) / 10));
    drawLine(xhand, yhand, clock[i * 5][0], clock[i * 5][1], thin);
  }

  drawLine(67, 32, clock[second][0], clock[second][1], thin);

  x1m = clock[minute][0] - (clock[minute][0] - 67) / 80;
  y1m = clock[minute][1] - (clock[minute][1] - 32) / 80;
  drawLine(67, 32, x1m, y1m, thick);

  x1h = 67 + (clock[((hour % 12) * 5)][0] - 67) / 80;
  x1h = 32 + (clock[((hour % 12) * 5)][1] - 32) / 80;
  drawLine(67, 32, x1h, x1h, fat);
}

void smallDate(void)
{
  sprintf(toprint, "%d/%d", day, month);
  ProtectedOledPutString(toprint, 15, 110, 1);
}

void smallAMPM(void)
{
  if (ampmMode == 1)
    sprintf(toprint, hour >= 12 ? "PM" : "AM");
  else
    sprintf(toprint, "24H");
  ProtectedOledPutString(toprint, 15, 0, 1);
}

void bigClock(void)
{
  char displayHour = hour;
  if (ampmMode == 1)
    displayHour = hour % 12 == 0 ? 12 : hour % 12;
  sprintf(toprint, "%d:%d:%d", displayHour, minute, second);
  ProtectedBigOledPutString(toprint, 10, 35, 0);
}

void smallClock(void)
{
  char displayHour = hour;
  if (ampmMode == 1)
    displayHour = hour % 12 == 0 ? 12 : hour % 12;
  sprintf(toprint, "%d:%d", displayHour, minute);
  ProtectedOledPutString(toprint, 0, 70, 1);
  if (ampmMode == 1)
    sprintf(toprint, hour >= 12 ? "PM" : "AM");
  else
    sprintf(toprint, "24H");
  ProtectedOledPutString(toprint, 0, 110, 1);
}

void showSteps(void)
{
  sprintf(toprint, "%d", currSteps);
  ProtectedOledPutString(toprint, 0, 20, 1);
}

void bigClockDisplay(void)
{
  analogClock();
  smallAMPM();
  smallDate();
  showSteps();
  icon();
  DelayMs(50); //to stop refresh rate
}

void bigDigitalClockDisplay(void)
{
  bigClock();
  smallAMPM();
  smallDate();
  showSteps();
  icon();
  DelayMs(50); //to stop refresh rate
}

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/

#define FOREGROUND

void main(void)
{
  InitializeSystem();

  T0CONbits.T08BIT = 0; //Timer0 16BIT COUNTER
  T0CONbits.T0CS = 0;   //Clock Source -- Internal
  T0CONbits.PSA = 0;    //Use Pre-Scaler
  T0CONbits.T0PS = 1;   //Prescale 1:4
  T0CONbits.TMR0ON = 1; //Set Timer to ON

  RCONbits.IPEN = 1;    //Use Priority Interrutps
  INTCON2bits.T0IP = 1; //Timer0 High-Priority

  INTCONbits.GIE = 1; //Enable Interrupts
  INTCONbits.PEIE = 1;
  INTCONbits.T0IE = 1; //Timer0 Overflow Interrupt Enabled

  while (1) //Main is Usualy an Endless Loop
  {
    clearScreen0();
#ifdef FOREGROUND

    if (analogMode == 0)
      bigDigitalClockDisplay();
    else
      bigClockDisplay();

    if (CheckButtonPressed())
      mainMenu();

#endif
  }
}

/** EOF main.c *************************************************/
//#endif
