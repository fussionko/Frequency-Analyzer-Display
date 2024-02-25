#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }

    virtual void UpdateFrequencyGraph(float* displayBuffer, const int size);

    virtual void UpdateGraphScaleMagnitude();
    virtual void UpdateGraphScaleDBFS();
    virtual void UpdateGraphScalePSD();

    virtual void UpdatePeakFrequency(float peakFrequency);
    virtual void UpdateLinearRMS(float linearRMS);
protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
