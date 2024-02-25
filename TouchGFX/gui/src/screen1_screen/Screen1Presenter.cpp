#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{
	m_FrameCountUpdatePeakFrequency = 0;
	m_FrameCountUpdateLinearRMS 	= 0;
}


void Screen1Presenter::UpdateGraphScale()
{
	model->UpdateGraphScale();
}

void Screen1Presenter::UpdateFrequencyGraph(float* displayBuffer, const int size)
{
	view.UpdateFrequencyGraph(displayBuffer, size);
}

void Screen1Presenter::UpdateWindowFunction(unsigned char windowFunction)
{
	model->UpdateWindowFunction(windowFunction);
}


void Screen1Presenter::UpdateGraphScaleMagnitude()
{
	view.UpdateGraphScaleMagnitude();
}

void Screen1Presenter::UpdateGraphScaleDBFS()
{
	view.UpdateGraphScaleDBFS();
}

void Screen1Presenter::UpdateGraphScalePSD()
{
	view.UpdateGraphScalePSD();
}

void Screen1Presenter::UpdatePeakFrequency(float peakFrequency)
{
    m_FrameCountUpdatePeakFrequency++;
    if(m_FrameCountUpdatePeakFrequency == TICK_UPDATEPEAKFREQUENCY_INTERVAL)
    {
    	view.UpdatePeakFrequency(peakFrequency);
    	m_FrameCountUpdatePeakFrequency = 0;
    }
}

void Screen1Presenter::UpdateLinearRMS(float linearRMS)
{
    m_FrameCountUpdateLinearRMS++;
    if(m_FrameCountUpdateLinearRMS == TICK_UPDATELINEARRMS_INTERVAL)
    {
    	view.UpdateLinearRMS(linearRMS);
    	m_FrameCountUpdateLinearRMS = 0;
    }
}


/*
void __attribute__((optimize("O0"))) Screen1Presenter::updateFrequencyGraph()
{
	view.updateFrequencyGraph();
}
*/
