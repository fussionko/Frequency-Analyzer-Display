# Frequency-Analyzer-Display
Frequency-Analyzer developed for STM32H750B-DK board. 
It takes the sound from on board MEMS microphone, which is then sampled by pdm2pcm library to convert from PDM to PCM.
Data is the converted from time domain to frequency domain representation with FFT, which is included from arm math library. 
Then it is sent to the display module via queue where it is displayed on the screen.
