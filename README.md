
# Frequency-Analyzer-Display
Frequency-Analyzer developed for STM32H750B-DK board.\
Cube IDE 1.14.0, TouchGFX Designer 4.23.0

It takes the sound from on board MEMS microphone, which is then sampled by pdm2pcm library to convert from PDM to PCM.
Data is the converted from time domain to frequency domain representation with FFT, which is included from arm math library. 
Then it is sent to the display module via queue where it is displayed on the screen.

On the screen is a option to select a window function, change data presentation and you can see selected settings and current info.

<pre>
Window function:
  -> Possible window functions: None, Hanning, Hamming, Flat top and Bartlett
  -> Window function is applied to PCM date before FFT happens.
  -> Because the function values are always the same, the values are calculated at the start of the programm. There is
     a index that stores which window function is currently used, which can then be used to calculate windowed data.
</pre>

<pre>
Display info:
  -> The size of the FFT which needs to be selected before build and needs to be changed in STM Designer (graph points)
  -> Bin size, this is calculated automatically based on sampling frequency and fft size
  -> Overlap percantage, this can be set before build
  -> Peak Hz
  -> RMS (Root Mean Squared) value
</pre>

<pre>
Data presentation:
  -> Normal magnitude
  -> dBFS in log scale
  -> PSD (Power Spectral Density) in log scale
</pre>

There is also an option for removing the average from PCM data before fft, which should remove DC offset, but it doesn't work
as intended.
![20240226_072013](https://github.com/fussionko/Frequency-Analyzer-Display/assets/36604107/0ceb5a8d-890e-421d-9472-9cca130f0207)



https://github.com/fussionko/Frequency-Analyzer-Display/assets/36604107/73acf9bf-2829-4a8c-b61b-6c34d07b611c


For better data vizualization a logarithmic scale should be implemented (currently STM Designer doesn't support it) and there
is a observable amount of leakage to other frequency bins.




  
