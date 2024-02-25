#ifndef __MAIN_DEF_H
#define __MAIN_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "my_audio_def.h"
#include "window_function_def.h"

#define FFT_BUFFER_SIZE 		1024
#define MAGNITUDE_BUFFER_SIZE 	FFT_BUFFER_SIZE / 2
#define FFT_BIN_FREQUENCY		((float)AUDIO_FREQUENCY / (float)FFT_BUFFER_SIZE)



#define DISPLAY_FFT_BUFFER 1

#define OVERLAP_FFT_BUFFER 		3
#define OVERLAP_FFT_SAMPLES		(FFT_BUFFER_SIZE - (uint16_t)(FFT_BUFFER_SIZE / OVERLAP_FFT_BUFFER))
#define OVERLAP_FFT_PERCENTAGE	((float)OVERLAP_FFT_SAMPLES / (float)FFT_BUFFER_SIZE)

#define FFT_MAGNITUDE_BUFFER_SCALING 1

//#define FFT_REMOVE_MEAN 1


typedef struct {
	float* magnitudeBufferSquared;
	float peakFrequency;
	float linearRMS;
} fft_data_t;


#ifdef FFT_MAGNITUDE_BUFFER_SCALING
extern float ScalingFFTBufferSum[NUM_WINDOW_FUNCTIONS];
extern float ScalingSquaredFFTBufferSum[NUM_WINDOW_FUNCTIONS];
#endif

// TEST
#define NUMBER_OF_LINES ((float)FFT_BUFFER_SIZE)
#define FFT_LINE_RESOLUTION (((float)AUDIO_FREQUENCY * 2.0) / NUMBER_OF_LINES)
#define FFT_TIME_TO_CALCULATE ((float)NUMBER_OF_LINES * 2.0 / (float)AUDIO_FREQUENCY)

#ifdef __cplusplus
}
#endif


#endif /* __MAIN_DEF_H */
