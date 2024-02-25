#ifndef __WINDOW_FUNCTION_H
#define __WINDOW_FUNCTION_H

#include "stm32h7xx_hal.h"


double WindowNone(uint16_t index, uint16_t bufferSize);
double WindowHanning(uint16_t index, uint16_t bufferSize);
double WindowHamming(uint16_t index, uint16_t bufferSize);
double WindowFlatTop(uint16_t index, uint16_t bufferSize);
double WindowBartlett(uint16_t index, uint16_t bufferSize);

#endif // __WINDOW_FUNCTION_H
