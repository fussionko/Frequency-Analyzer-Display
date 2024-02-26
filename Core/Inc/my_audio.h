#ifndef _MY_AUDIO_H
#define _MY_AUDIO_H

#include "stm32h7xx_hal.h"

#include "my_audio_def.h"



typedef struct {
	uint32_t 	SampleRate;
	uint8_t 	Volume;
	uint8_t 	Channels;
	uint8_t 	BitsPerSample;
}Audio_t;
extern Audio_t Audio;

extern uint16_t PDMBuffer[AUDIO_PDM_BUFFER_SIZE];
extern uint16_t PCMBuffer[AUDIO_PCM_BUFFER_SIZE];
extern volatile uint16_t PCMBufferIndex;

HAL_StatusTypeDef MyAudioInit(void);
void PDMToPCM(uint16_t* PDMBuf, uint16_t* PCMBuf);

#endif 	//_MY_AUDIO_H
