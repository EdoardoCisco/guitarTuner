#ifndef __SCREENFUNCTION_H__
#define __SCREENFUNCTION_H__

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

void initScreen();
void cleanScreen();

void printChoice();
void selectOption(uint8_t);

void metronomeBPMValue(uint8_t *);
void selectedDigit(uint8_t);
int incrementBPMvalue(uint8_t);

void drawBar(uint8_t *);

#endif
