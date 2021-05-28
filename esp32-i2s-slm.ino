/*
 * Display A-weighted sound level measured by I2S Microphone
 * 
 * (c)2019 Ivan Kostoski
 * (c)2021 Bim Overbohm (split into files, template)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *    
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Sketch samples audio data from I2S microphone, processes the data 
 * with digital IIR filters and calculates A or C weighted Equivalent 
 * Continuous Sound Level (Leq)
 * 
 * I2S is setup to sample data at Fs=48000KHz (fixed value due to 
 * design of digital IIR filters). Data is read from I2S queue 
 * in 'sample blocks' (default 125ms block, equal to 6000 samples) 
 * by 'i2s_reader_task', filtered trough two IIR filters (equalizer 
 * and weighting), summed up and pushed into 'samples_queue' as 
 * sum of squares of filtered samples. The main task then pulls data 
 * from the queue and calculates decibel value relative to microphone 
 * reference amplitude, derived from datasheet sensitivity dBFS 
 * value, number of bits in I2S data, and the reference value for 
 * which the sensitivity is specified (typically 94dB, pure sine
 * wave at 1KHz).
 * 
 * Displays line on the small OLED screen with 'short' LAeq(125ms)
 * response and numeric LAeq(1sec) dB value from the signal RMS.
 */

#include "i2s_mic.h"
#include "filters.h"

//
// Configuration
//

#define LEQ_PERIOD 1          // second(s)
#define WEIGHTING C_weighting // Also avaliable: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS "LAeq"      // customize based on above weighting used
#define DB_UNITS "dBA"        // customize based on above weighting used
#define USE_DISPLAY 1

// NOTE: Some microphones require at least DC-Blocker filter
#define MIC_EQUALIZER ICS43434 // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB 3.0103   // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

// Customize these values from microphone datasheet
#define MIC_SENSITIVITY -26   // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB 94.0       // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB 116.0 // dB - Acoustic overload point
#define MIC_NOISE_DB 29       // dB - Noise floor
#define MIC_BITS 24           // valid number of bits in I2S data
#define MIC_CONVERT(s) (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT false // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY) / 20) * ((1 << (MIC_BITS - 1)) - 1);

#define SAMPLE_RATE_HZ 48000 // Hz, fixed to design of IIR filters. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
#define SAMPLE_COUNT 2048    // ~40ms sample time, must be power-of-two
// Static buffer for block of samples
float samples[SAMPLE_COUNT] __attribute__((aligned(4)));

//
// I2S pins - Can be routed to almost any (unused) ESP32 pin.
//            SD can be any pin, inlcuding input only pins (36-39).
//            SCK (i.e. BCLK) and WS (i.e. L/R CLK) must be output capable pins
//
// Below ones are just example for my board layout, put here the pins you will use
//
#define I2S_WS 18
#define I2S_SCK 23
#define I2S_SD 19

// I2S peripheral to use (0 or 1)
#define I2S_PORT I2S_NUM_0

// Set up microphone
auto mic = Microphone_I2S<SAMPLE_COUNT, I2S_WS, I2S_SCK, I2S_SD, I2S_PORT, MIC_BITS, MIC_TIMING_SHIFT, SAMPLE_RATE_HZ>(MIC_EQUALIZER);

//
// Setup your display library (and geometry) here
//
#if (USE_DISPLAY > 0)
// ThingPulse/esp8266-oled-ssd1306, you may need the latest source and PR#198 for 64x48
#include <SSD1306Wire.h>
#define OLED_GEOMETRY GEOMETRY_64_48
//#define OLED_GEOMETRY GEOMETRY_128_32
//#define OLED_GEOMETRY GEOMETRY_128_64
#define OLED_FLIP_V 1
SSD1306Wire display(0x3c, SDA, SCL, OLED_GEOMETRY);
#endif

//
// Setup and main loop
//
// Note: Use doubles, not floats, here unless you want to pin
//       the task to whichever core it happens to run on at the moment
//
void setup()
{
  // If needed, now you can actually lower the CPU frquency,
  // i.e. if you want to (slightly) reduce ESP32 power consumption
  setCpuFrequencyMhz(80); // It should run as low as 80MHz

  Serial.begin(115200);
  delay(1000); // Safety

#if (USE_DISPLAY > 0)
  display.init();
#if (OLED_FLIP_V > 0)
  display.flipScreenVertically();
#endif
  display.setFont(ArialMT_Plain_16);
#endif

  mic.begin();
  Serial.println("Starting sampling from mic");
  mic.startSampling();

  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  double Leq_dB = 0;

  // Read equalized samples read from microphone
  while (xQueueReceive(mic.sampleQueue(), &samples, portMAX_DELAY))
  {

    // Sum of squares of mic equalized samples
    float sum_sqr_SPL = MIC_EQUALIZER.sum_of_squares(samples, samples, SAMPLE_COUNT);
    // Sum of squares of equalized + weighted mic samples
    float sum_sqr_weighted = WEIGHTING.sum_of_squares(samples, samples, SAMPLE_COUNT);

    // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
    double short_RMS = sqrt(double(sum_sqr_SPL) / SAMPLE_COUNT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // In case of acoustic overload or below noise floor measurement, report infinty Leq value
    if (short_SPL_dB > MIC_OVERLOAD_DB)
    {
      Leq_sum_sqr = INFINITY;
    }
    else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB))
    {
      Leq_sum_sqr = -INFINITY;
    }

    // Accumulate Leq sum
    Leq_sum_sqr += sum_sqr_weighted;
    Leq_samples += SAMPLE_COUNT;

    // When we gather enough samples, calculate new Leq value
    if (Leq_samples >= SAMPLE_RATE_HZ * LEQ_PERIOD)
    {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;

      // Serial output, customize (or remove) as needed
      Serial.printf("%.1f\n", Leq_dB);

      // Debug only
      //Serial.printf("%u processing ticks\n", q.proc_ticks);
    }

#if (USE_DISPLAY > 0)

    //
    // Example code that displays the measured value.
    // You should customize the below code for your display
    // and display library used.
    //

    display.clear();

    // It is important to somehow notify when the deivce is out of its range
    // as the calculated values are very likely with big error
    if (Leq_dB > MIC_OVERLOAD_DB)
    {
      // Display 'Overload' if dB value is over the AOP
      display.drawString(0, 24, "Overload");
    }
    else if (isnan(Leq_dB) || (Leq_dB < MIC_NOISE_DB))
    {
      // Display 'Low' if dB value is below noise floor
      display.drawString(0, 24, "Low");
    }

    // The 'short' Leq line
    double short_Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(double(sum_sqr_weighted) / SAMPLE_COUNT) / MIC_REF_AMPL);
    uint16_t len = min(max(0, int(((short_Leq_dB - MIC_NOISE_DB) / MIC_OVERLOAD_DB) * (display.getWidth() - 1))), display.getWidth() - 1);
    display.drawHorizontalLine(0, 0, len);
    display.drawHorizontalLine(0, 1, len);
    display.drawHorizontalLine(0, 2, len);

    // The Leq numeric decibels
    display.drawString(0, 4, String(Leq_dB, 1) + " " + DB_UNITS);

    display.display();

#endif // USE_DISPLAY
  }
}

void loop()
{
  // Nothing here..
}
