# Frequency-Analyzer-Display
Frequency-Analyzer developed for STM32H750B-DK board. 
It takes the sound from on board MEMS microphone, which is then sampled by pdm2pcm library to convert from PDM to PCM.
Data is the converted from time domain to frequency domain representation with FFT, which is included from arm math library. 
Then it is sent to the display module via queue where it is displayed on the screen.

On the screen is a option to select a window function, change data presentation and you can see selected settings and current info.

Window function:
  -> Possible window functions: None, Hanning, Hamming, Flat top and Bartlett
  -> Window function is applied to PCM date before FFT happens.
  -> Because the function values are always the same, the values are calculated at the start of the programm. There is
     a index that stores which window function is currently used, which can then be used to calculate windowed data.

Display info:
  -> The size of the FFT which needs to be selected before build and needs to be changed in STM Designer (graph points)
  -> Bin size, this is calculated automatically based on sampling frequency and fft size
  -> Overlap percantage, this can be set before build
  -> Peak Hz
  -> RMS (Root Mean Squared) value

  
