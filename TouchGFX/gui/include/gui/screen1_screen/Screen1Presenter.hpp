#ifndef SCREEN1PRESENTER_HPP
#define SCREEN1PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

#define TICK_UPDATEPEAKFREQUENCY_INTERVAL 5
#define TICK_UPDATELINEARRMS_INTERVAL 5

class Screen1View;

class Screen1Presenter : public touchgfx::Presenter, public ModelListener
{
public:
    Screen1Presenter(Screen1View& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~Screen1Presenter() {}

    virtual void UpdateFrequencyGraph(float* displayBuffer, const int size);
    virtual void UpdateWindowFunction(unsigned char windowFunction);

    virtual void UpdateGraphScale();

    virtual void UpdateGraphScaleMagnitude();
    virtual void UpdateGraphScaleDBFS();
    virtual void UpdateGraphScalePSD();

    void UpdatePeakFrequency(float peakFrequency);
    void UpdateLinearRMS(float linearRMS);
private:
    Screen1Presenter();

    Screen1View& view;

    int m_FrameCountUpdatePeakFrequency;
    int m_FrameCountUpdateLinearRMS;
};

#endif // SCREEN1PRESENTER_HPP
