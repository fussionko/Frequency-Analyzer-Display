/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <touchgfx/canvas_widget_renderer/CanvasWidgetRenderer.hpp>
#include <touchgfx/Color.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

Screen1ViewBase::Screen1ViewBase() :
    graphClickedCallback(this, &Screen1ViewBase::graphClickedCallbackHandler)
{
    touchgfx::CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);
    
    __background.setPosition(0, 0, 480, 272);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(__background);

    tiledImage1.setBitmap(touchgfx::Bitmap(BITMAP_DARK_THEME_IMAGES_CONTAINERS_LARGE_WIDE_DARK_ID));
    tiledImage1.setPosition(0, 0, 480, 272);
    tiledImage1.setOffset(0, 0);
    add(tiledImage1);

    FrequencyGraph.setPosition(15, 25, 355, 215);
    FrequencyGraph.setScaleX(1);
    FrequencyGraph.setScaleY(1);
    FrequencyGraph.setGraphAreaMargin(15, 23, 0, 15);
    FrequencyGraph.setGraphAreaPadding(0, 0, 0, 0);
    FrequencyGraph.setGraphRangeX(0, 23000);
    FrequencyGraph.setGraphRangeY(0, 1000);
    FrequencyGraph.setClickAction(graphClickedCallback);
    FrequencyGraphMajorXAxisGrid.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    FrequencyGraphMajorXAxisGrid.setInterval(5000);
    FrequencyGraphMajorXAxisGrid.setLineWidth(1);
    FrequencyGraphMajorXAxisGrid.setScale(1);
    FrequencyGraph.addGraphElement(FrequencyGraphMajorXAxisGrid);

    FrequencyGraphMajorYAxisGrid.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    FrequencyGraphMajorYAxisGrid.setInterval(100);
    FrequencyGraphMajorYAxisGrid.setLineWidth(1);
    FrequencyGraphMajorYAxisGrid.setScale(1);
    FrequencyGraph.addGraphElement(FrequencyGraphMajorYAxisGrid);

    FrequencyGraphMajorXAxisLabel.setInterval(5000);
    FrequencyGraphMajorXAxisLabel.setLabelTypedText(touchgfx::TypedText(T___SINGLEUSE_JUHU));
    FrequencyGraphMajorXAxisLabel.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    FrequencyGraphMajorXAxisLabel.setScale(1);
    FrequencyGraph.addBottomElement(FrequencyGraphMajorXAxisLabel);

    FrequencyGraphMajorYAxisLabel.setInterval(100);
    FrequencyGraphMajorYAxisLabel.setLabelTypedText(touchgfx::TypedText(T___SINGLEUSE_R6SY));
    FrequencyGraphMajorYAxisLabel.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    FrequencyGraphMajorYAxisLabel.setScale(1);
    FrequencyGraph.addLeftElement(FrequencyGraphMajorYAxisLabel);

    FrequencyGraphLine1Painter.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    FrequencyGraphLine1.setPainter(FrequencyGraphLine1Painter);
    FrequencyGraphLine1.setLineWidth(2);
    FrequencyGraph.addGraphElement(FrequencyGraphLine1);


    add(FrequencyGraph);

    buttonFlatTop.setXY(383, 112);
    buttonFlatTop.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_PRESSED_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_DARK_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_PRESSED_ID));
    buttonFlatTop.setSelected(false);
    buttonFlatTop.setDeselectionEnabled(false);
    windowButtonGroup.add(buttonFlatTop);
    add(buttonFlatTop);

    buttonBartlett.setXY(383, 141);
    buttonBartlett.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_PRESSED_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_DARK_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_PRESSED_ID));
    buttonBartlett.setSelected(false);
    buttonBartlett.setDeselectionEnabled(false);
    windowButtonGroup.add(buttonBartlett);
    add(buttonBartlett);

    buttonHamming.setXY(383, 83);
    buttonHamming.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_PRESSED_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_DARK_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_PRESSED_ID));
    buttonHamming.setSelected(false);
    buttonHamming.setDeselectionEnabled(false);
    windowButtonGroup.add(buttonHamming);
    add(buttonHamming);

    buttonHanning.setXY(383, 54);
    buttonHanning.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_PRESSED_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_DARK_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_PRESSED_ID));
    buttonHanning.setSelected(false);
    buttonHanning.setDeselectionEnabled(false);
    windowButtonGroup.add(buttonHanning);
    add(buttonHanning);

    buttonNone.setXY(383, 25);
    buttonNone.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_OFF_PRESSED_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_DARK_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_SMALL_ROUND_ON_PRESSED_ID));
    buttonNone.setSelected(true);
    buttonNone.setDeselectionEnabled(false);
    windowButtonGroup.add(buttonNone);
    add(buttonNone);

    textAreaHanning.setXY(415, 60);
    textAreaHanning.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaHanning.setLinespacing(0);
    textAreaHanning.setTypedText(touchgfx::TypedText(T___SINGLEUSE_7038));
    add(textAreaHanning);

    textAreaNone.setXY(415, 31);
    textAreaNone.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaNone.setLinespacing(0);
    textAreaNone.setTypedText(touchgfx::TypedText(T___SINGLEUSE_P6AU));
    add(textAreaNone);

    textAreaHamming.setXY(415, 89);
    textAreaHamming.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaHamming.setLinespacing(0);
    textAreaHamming.setTypedText(touchgfx::TypedText(T___SINGLEUSE_U7PY));
    add(textAreaHamming);

    textAreaFlatTop.setXY(415, 118);
    textAreaFlatTop.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaFlatTop.setLinespacing(0);
    textAreaFlatTop.setTypedText(touchgfx::TypedText(T___SINGLEUSE_NSL4));
    add(textAreaFlatTop);

    textAreaBartlett.setXY(415, 144);
    textAreaBartlett.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaBartlett.setLinespacing(0);
    textAreaBartlett.setTypedText(touchgfx::TypedText(T___SINGLEUSE_ETPW));
    add(textAreaBartlett);

    textAreaWindowFunction.setXY(377, 7);
    textAreaWindowFunction.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaWindowFunction.setLinespacing(0);
    textAreaWindowFunction.setTypedText(touchgfx::TypedText(T___SINGLEUSE_W8QQ));
    add(textAreaWindowFunction);

    textAreaGraphLabelX.setXY(180, 240);
    textAreaGraphLabelX.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaGraphLabelX.setLinespacing(0);
    textAreaGraphLabelX.setTypedText(touchgfx::TypedText(T___SINGLEUSE_JHF8));
    add(textAreaGraphLabelX);

    textAreaGraphLabelY.setPosition(3, 99, 12, 85);
    textAreaGraphLabelY.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaGraphLabelY.setLinespacing(0);
    textAreaGraphLabelY.setRotation(touchgfx::TEXT_ROTATE_270);
    Unicode::snprintf(textAreaGraphLabelYBuffer, TEXTAREAGRAPHLABELY_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_T3HA).getText());
    textAreaGraphLabelY.setWildcard(textAreaGraphLabelYBuffer);
    textAreaGraphLabelY.setTypedText(touchgfx::TypedText(T___SINGLEUSE_GJ4M));
    add(textAreaGraphLabelY);

    textAreaOverlap.setPosition(377, 218, 100, 12);
    textAreaOverlap.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaOverlap.setLinespacing(0);
    Unicode::snprintf(textAreaOverlapBuffer, TEXTAREAOVERLAP_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_TVYC).getText());
    textAreaOverlap.setWildcard(textAreaOverlapBuffer);
    textAreaOverlap.setTypedText(touchgfx::TypedText(T___SINGLEUSE_7E9B));
    add(textAreaOverlap);

    textAreaFFT.setXY(389, 176);
    textAreaFFT.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaFFT.setLinespacing(0);
    textAreaFFT.setTypedText(touchgfx::TypedText(T___SINGLEUSE_PROO));
    add(textAreaFFT);

    textAreaWindowSize.setPosition(377, 193, 100, 12);
    textAreaWindowSize.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaWindowSize.setLinespacing(0);
    Unicode::snprintf(textAreaWindowSizeBuffer, TEXTAREAWINDOWSIZE_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_C0CS).getText());
    textAreaWindowSize.setWildcard(textAreaWindowSizeBuffer);
    textAreaWindowSize.setTypedText(touchgfx::TypedText(T___SINGLEUSE_TYV9));
    add(textAreaWindowSize);

    textAreaPeakFrequency.setPosition(377, 231, 100, 12);
    textAreaPeakFrequency.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaPeakFrequency.setLinespacing(0);
    Unicode::snprintf(textAreaPeakFrequencyBuffer, TEXTAREAPEAKFREQUENCY_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_B60I).getText());
    textAreaPeakFrequency.setWildcard(textAreaPeakFrequencyBuffer);
    textAreaPeakFrequency.setTypedText(touchgfx::TypedText(T___SINGLEUSE_24UL));
    add(textAreaPeakFrequency);

    textAreaBin.setPosition(377, 205, 100, 12);
    textAreaBin.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaBin.setLinespacing(0);
    Unicode::snprintf(textAreaBinBuffer, TEXTAREABIN_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_7WP1).getText());
    textAreaBin.setWildcard(textAreaBinBuffer);
    textAreaBin.setTypedText(touchgfx::TypedText(T___SINGLEUSE_WAMO));
    add(textAreaBin);

    textAreaLinearRMS.setPosition(377, 244, 100, 12);
    textAreaLinearRMS.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textAreaLinearRMS.setLinespacing(0);
    Unicode::snprintf(textAreaLinearRMSBuffer, TEXTAREALINEARRMS_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_PA58).getText());
    textAreaLinearRMS.setWildcard(textAreaLinearRMSBuffer);
    textAreaLinearRMS.setTypedText(touchgfx::TypedText(T___SINGLEUSE_L289));
    add(textAreaLinearRMS);
}

Screen1ViewBase::~Screen1ViewBase()
{
    touchgfx::CanvasWidgetRenderer::resetBuffer();
}

void Screen1ViewBase::setupScreen()
{

}

void Screen1ViewBase::graphClickedCallbackHandler(const touchgfx::AbstractDataGraph& src, const touchgfx::AbstractDataGraph::GraphClickEvent& value)
{
    if (&src == &FrequencyGraph)
    {
        //ChangeGraph
        //When FrequencyGraph clicked call virtual function
        //Call ChangeGraph
        ChangeGraph(value);
    }
}