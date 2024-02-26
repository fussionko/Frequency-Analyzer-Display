#include <gui/screen1_screen/Screen1View.hpp>

#include "main_def.h"
#include "my_audio_def.h"

#define GRAPH_SCALE_MAGNITUDE_MIN 	0
#define GRAPH_SCALE_MAGNITUDE_MAX 	1000
#define GRAPH_SCALE_DBFS_MIN 		-150
#define GRAPH_SCALE_DBFS_MAX 		0
#define GRAPH_SCALE_PSD_MIN 		-150
#define GRAPH_SCALE_PSD_MAX 		0

#define GRAPH_SCALE_MAGNITUDE_INTERVAL 	100
#define GRAPH_SCALE_DBFS_INTERVAL 		15
#define GRAPH_SCALE_PSD_INTERVAL 		15



Screen1View::Screen1View() : cbRadioBtnGroup(this, &Screen1View::cbRadioBtnGroupHandler)
{
    windowButtonGroup.setRadioButtonSelectedHandler(cbRadioBtnGroup);

    Unicode::snprintf(textAreaWindowSizeBuffer, TEXTAREAWINDOWSIZE_SIZE, "%d", FFT_BUFFER_SIZE);
    textAreaWindowSize.invalidate();

    Unicode::snprintfFloat(textAreaBinBuffer, TEXTAREABIN_SIZE, "%-.3f", FFT_BIN_FREQUENCY);
    textAreaBin.invalidate();

    Unicode::snprintfFloat(textAreaOverlapBuffer, TEXTAREAOVERLAP_SIZE, "%-.2f", OVERLAP_FFT_PERCENTAGE * 100.0f);
    textAreaOverlap.invalidate();
}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}


void Screen1View::ChangeGraph(AbstractDataGraph::GraphClickEvent value)
{
	if (value.clickEvent.getType() == touchgfx::ClickEvent::PRESSED)
	{
		presenter->UpdateGraphScale();
		//FrequencyGraph = touchgfx::Graph<512>;
	}
}

void Screen1View::cbRadioBtnGroupHandler(const touchgfx::AbstractButton& src)
{
	unsigned char windowFunction = WINDOW_FUNCTION_NONE;

	if (&src == &buttonHanning) 		windowFunction = WINDOW_FUNCTION_HANNING;
	else if (&src == &buttonHamming) 	windowFunction = WINDOW_FUNCTION_HAMMING;
	else if (&src == &buttonFlatTop) 	windowFunction = WINDOW_FUNCTION_FLATTOP;
	else if (&src == &buttonBartlett) 	windowFunction = WINDOW_FUNCTION_BARTLETT;

	presenter->UpdateWindowFunction(windowFunction);
}

void Screen1View::UpdateFrequencyGraph(float* displayBuffer, const int size)
{
	if (displayBuffer == NULL)
		return;

	for (int i = 0; i < size; ++i)
	{
		FrequencyGraph.addDataPoint(FFT_BIN_FREQUENCY * (float)i, displayBuffer[i]);
	}
}

void Screen1View::UpdateGraphScaleMagnitude()
{
	FrequencyGraph.setGraphRangeY(GRAPH_SCALE_MAGNITUDE_MIN, GRAPH_SCALE_MAGNITUDE_MAX);

    FrequencyGraphMajorYAxisGrid.setInterval(GRAPH_SCALE_MAGNITUDE_INTERVAL);
    FrequencyGraphMajorYAxisLabel.setInterval(GRAPH_SCALE_MAGNITUDE_INTERVAL);

    Unicode::strncpy(textAreaGraphLabelYBuffer, "Magnitude", TEXTAREAGRAPHLABELY_SIZE);
    textAreaGraphLabelY.invalidate();
}

void Screen1View::UpdateGraphScaleDBFS()
{
	FrequencyGraph.setGraphRangeY(GRAPH_SCALE_DBFS_MIN, GRAPH_SCALE_DBFS_MAX);

    FrequencyGraphMajorYAxisGrid.setInterval(GRAPH_SCALE_DBFS_INTERVAL);
    FrequencyGraphMajorYAxisLabel.setInterval(GRAPH_SCALE_DBFS_INTERVAL);

    Unicode::strncpy(textAreaGraphLabelYBuffer, "dBFS", TEXTAREAGRAPHLABELY_SIZE);
    textAreaGraphLabelY.invalidate();
}

void Screen1View::UpdateGraphScalePSD()
{
	FrequencyGraph.setGraphRangeY(GRAPH_SCALE_PSD_MIN, GRAPH_SCALE_PSD_MAX);

    FrequencyGraphMajorYAxisGrid.setInterval(GRAPH_SCALE_PSD_INTERVAL);
    FrequencyGraphMajorYAxisLabel.setInterval(GRAPH_SCALE_PSD_INTERVAL);

    Unicode::strncpy(textAreaGraphLabelYBuffer, "PSD (dBm/Hz)", TEXTAREAGRAPHLABELY_SIZE);
    textAreaGraphLabelY.invalidate();
}


void Screen1View::UpdatePeakFrequency(float peakFrequency)
{
    Unicode::snprintfFloat(textAreaPeakFrequencyBuffer, TEXTAREAPEAKFREQUENCY_SIZE, "%-.3f", peakFrequency / 1000.0f);
    textAreaPeakFrequency.setWildcard(textAreaPeakFrequencyBuffer);
    textAreaPeakFrequency.invalidate();
}

void Screen1View::UpdateLinearRMS(float linearRMS)
{
    Unicode::snprintfFloat(textAreaLinearRMSBuffer, TEXTAREALINEARRMS_SIZE, "%-.3f", linearRMS);
    textAreaLinearRMS.setWildcard(textAreaLinearRMSBuffer);
    textAreaLinearRMS.invalidate();
}

