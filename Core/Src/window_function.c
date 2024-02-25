#include "window_function.h"
#include <math.h>

// Sources: Matlab

#define WINDOW_CALC(index, size) ((2.0 * M_PI * index) / (double)(size - 1))

#define WINDOW_NONE 1.0

#define WINDOW_HANNING_A0 0.5

#define WINDOW_HAMMING_A0 0.54
#define WINDOW_HAMMING_A1 0.46

#define WINDOW_FLAT_TOP_A0 0.21557895
#define WINDOW_FLAT_TOP_A1 0.41663158
#define WINDOW_FLAT_TOP_A2 0.277263158
#define WINDOW_FLAT_TOP_A3 0.083578947
#define WINDOW_FLAT_TOP_A4 0.006947368

#define WINDOW_BARLETT_CALC(index, size) (2.0 * (double)index / (double)(size - 1))

double WindowNone(uint16_t index, uint16_t bufferSize)
{
	UNUSED(index);
	UNUSED(bufferSize);
	return 1.0f;
}

double WindowHanning(uint16_t index, uint16_t bufferSize)
{
	return WINDOW_HANNING_A0 * (1.0 - cos(WINDOW_CALC(index, bufferSize)));
}

double WindowHamming(uint16_t index, uint16_t bufferSize)
{
	return WINDOW_HAMMING_A0 - WINDOW_HAMMING_A1 * cos(WINDOW_CALC(index, bufferSize));
}

double WindowFlatTop(uint16_t index, uint16_t bufferSize)
{
	return WINDOW_FLAT_TOP_A0 - WINDOW_FLAT_TOP_A1 * cos(WINDOW_CALC(index, bufferSize))
			  + WINDOW_FLAT_TOP_A2 * cos(4.0 * WINDOW_CALC(index, bufferSize))
			  - WINDOW_FLAT_TOP_A3 * cos(6.0 * WINDOW_CALC(index, bufferSize))
			  + WINDOW_FLAT_TOP_A4 * cos(8.0 * WINDOW_CALC(index, bufferSize));
}

double WindowBartlett(uint16_t index, uint16_t bufferSize)
{
	return (index <= (double)bufferSize / 2.0) ?
			WINDOW_BARLETT_CALC(index, bufferSize) :
			2.0 - WINDOW_BARLETT_CALC(index, bufferSize);
}
