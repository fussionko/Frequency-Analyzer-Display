#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void UpdateFrequencyGraph(float* displayBuffer, const int size);


    virtual void ChangeGraph(AbstractDataGraph::GraphClickEvent value);

    void UpdateGraphScaleMagnitude();
    void UpdateGraphScaleDBFS();
    void UpdateGraphScalePSD();

    void UpdatePeakFrequency(float peakFrequency);
    void UpdateLinearRMS(float linearRMS);
protected:
    touchgfx::Callback<Screen1View, const touchgfx::AbstractButton&> cbRadioBtnGroup;
    void cbRadioBtnGroupHandler(const touchgfx::AbstractButton& src);

private:
    void TryDraw();

};

#endif // SCREEN1VIEW_HPP
