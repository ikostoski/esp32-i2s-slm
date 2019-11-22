# Sound Level Meter with Arduino IDE, ESP32 and I<sup>2</sup>S MEMS microphone

Arduino/ESP32 Sound Level Meter (SLM) using inexpensive, factory calibrated, digital I2S MEMS microphone and digital IIR filters (vs. FFT) for equalization and A-weighting.

The basic idea is:

![Basic principle](./misc/esp32-i2s-slm-bp.svg)

And the microphone response after equalization should look like:

![Adjusted frequency response](./misc/ics-43434-afr.svg)

Theoretically, i.e. with factory calibrated ICS-4343x, this should get you ±1dB(A) measurement within 20Hz-20KHz range.

The code in this repository is mostly intended as example how you can integrate resonable noise measurement (i.e. *L*<sub>Aeq</sub>, Equivalent Continuous Sound Level) in your projects. 

You can find a bit more information in my [hackday.io](https://hackaday.io/project/166867-esp32-i2s-slm) project.
