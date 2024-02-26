#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#include <cstring>
#include <cmath>

#include "cmsis_os.h"

#define PSD_BUFFER_SCALE 1000.0f

#define SET_DISPLAY_BUFFER(index, value) m_DisplayBuffer[index - 1] = value

extern "C"
{
	extern osMessageQueueId_t DisplayQueueHandle;
	extern osMessageQueueId_t UpdateQueueHandle;
	extern float ScalingFFTBufferSum[NUM_WINDOW_FUNCTIONS];
	extern float ScalingSquaredFFTBufferSum[NUM_WINDOW_FUNCTIONS];
}



Model::Model() : modelListener(0)
{
	m_GraphScaleIndex = 0;
}

void Model::UpdateGraphScale()
{
	// Update index
	m_GraphScaleIndex++;
	if (m_GraphScaleIndex == GRAPH_SCALE_STATES_SIZE)
	{
		m_GraphScaleIndex = 0;
	}

	switch (m_GraphScaleStates[m_GraphScaleIndex])
	{
	case GraphScaleState::MAGNITUDE: 	modelListener->UpdateGraphScaleMagnitude();	break;
	case GraphScaleState::DBFS: 		modelListener->UpdateGraphScaleDBFS(); 		break;
	case GraphScaleState::PSD: 			modelListener->UpdateGraphScalePSD(); 		break;
	}
}
//__attribute__((optimize("O0")))

void Model::tick()
{
	fft_data_t fftData;

	volatile osStatus_t status = osMessageQueueGet(DisplayQueueHandle, &fftData, NULL, 0U);
	if (status == osOK)
	{
		switch (m_GraphScaleStates[m_GraphScaleIndex])
		{
		case GraphScaleState::MAGNITUDE:	ProcessMagnitudeBuffer(fftData.magnitudeBufferSquared); break;
		case GraphScaleState::DBFS: 		ProcessDBFSBuffer(fftData.magnitudeBufferSquared); 		break;
		case GraphScaleState::PSD: 			ProcessPSDBuffer(fftData.magnitudeBufferSquared); 		break;
		}

		modelListener->UpdateFrequencyGraph(m_DisplayBuffer, MAGNITUDE_BUFFER_SIZE);
		modelListener->UpdatePeakFrequency(fftData.peakFrequency);
		modelListener->UpdateLinearRMS(fftData.linearRMS);
	}
}

void Model::UpdateWindowFunction(unsigned char windowFunction)
{
	m_SelectedWindowFunction = windowFunction;
	osMessageQueuePut(UpdateQueueHandle, &m_SelectedWindowFunction, 0U, 0U);
}

void Model::ProcessDBFSBuffer(float* magnitudeBuffer)
{
	for (uint16_t i = 1; i < MAGNITUDE_BUFFER_SIZE; i++)
	{
		//20log10(A/Max)
		SET_DISPLAY_BUFFER(i, 20.0f * log10f(sqrtf(magnitudeBuffer[i]) * 2.0f / ScalingFFTBufferSum[m_SelectedWindowFunction]));
	}
}

void Model::ProcessMagnitudeBuffer(float* magnitudeBuffer)
{
	for (uint16_t i = 1; i < MAGNITUDE_BUFFER_SIZE; i++)
	{
		SET_DISPLAY_BUFFER(i, sqrtf(magnitudeBuffer[i]));
	}
}

void Model::ProcessPSDBuffer(float* magnitudeBuffer)
{
	for (uint16_t i = 1; i < MAGNITUDE_BUFFER_SIZE; i++)
	{
		const float psd = magnitudeBuffer[i] / (ScalingSquaredFFTBufferSum[m_SelectedWindowFunction] * (float)AUDIO_FREQUENCY);
		SET_DISPLAY_BUFFER(i, 10.0f * log10f(2.0f * psd));
	}
}



