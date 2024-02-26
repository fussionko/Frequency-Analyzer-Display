#ifndef MODEL_HPP
#define MODEL_HPP

#include "main_def.h"

#define GRAPH_SCALE_STATES_SIZE 3

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();

    void UpdateWindowFunction(unsigned char windowFunction);
    void UpdateGraphScale();

protected:
    ModelListener* modelListener;

    float m_DisplayBuffer[MAGNITUDE_BUFFER_SIZE - 1] = {0};

    unsigned char m_SelectedWindowFunction = WINDOW_FUNCTION_NONE;

private:
    void ProcessDBFSBuffer(float* magnitudeBuffer);
    void ProcessMagnitudeBuffer(float* magnitudeBuffer);
    void ProcessPSDBuffer(float* magnitudeBuffer);

    enum GraphScaleState { MAGNITUDE, DBFS, PSD};
    GraphScaleState m_GraphScaleStates[GRAPH_SCALE_STATES_SIZE] = { MAGNITUDE, DBFS, PSD };
    int m_GraphScaleIndex;
};

#endif // MODEL_HPP
