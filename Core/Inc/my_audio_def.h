#ifndef _MY_AUDIO_DEF_H
#define _MY_AUDIO_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_FREQUENCY			48000U
#define AUDIO_VOLUME			80

#define AUDIO_PDM_BUFFER_SIZE  	(uint32_t)768
#define PDM2PCM_SAMPLES			48
#define PDM2PCM_SAMPLES_NUMBER	PDM2PCM_SAMPLES * 2		// 48 * 2  left and right
#define AUDIO_PCM_BUFFER_SIZE  	PDM2PCM_SAMPLES_NUMBER * 4
#define AUDIO_PCM_BUFFER_INDEX_MOVE PDM2PCM_SAMPLES_NUMBER

#define PDM2PCM_HIGHPASS_FILTER_COEFFICIENT 0.85 // coeff * (2^31 - 1)
#define PDM2PCM_MIC_GAIN 24

#define AUDIO_FRAME_SIZE 32768

#define AUDIO_PDM_STATE_NONE	0x0
#define AUDIO_PDM_STATE_HALF	0x1
#define AUDIO_PDM_STATE_FULL	0x2

#ifdef __cplusplus
}
#endif

#endif 	//_MY_AUDIO_DEF_H
