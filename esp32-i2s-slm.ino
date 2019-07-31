/*
 * Display A-weighted sound level measured by I2S Microphone
 * 
 * (c)2019 Ivan Kostoski
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
 * with digital IIR filters and calculates A-Weighted noise value.
 * 
 * I2S is setup to sample data at Fs=48000KHz sampling rate 
 * (fixed value due to design of digital IIR filters).
 * Data is read from I2S queue in 'sample blocks' (default 
 * 125ms block, equal to 6000 samples) by 'i2s_reader_task' 
 * and pushed into 'samples_queue' as sum of squares of filtered 
 * samples. The main task then pulls data from the queue and
 * calculated decibel value relative to microphone reference
 * amplitude at 94dB for 1Khz pure sine-wave (values from datasheet). 
 * Displays line on the small OLED screen with 'fast' (125ms) response 
 * and numeric dBA value with 'slow' (1sec) from signal RMS.
 */ 

// ESP32 peripherals
#include <driver/i2s.h>
#include <soc/rtc.h>

// ThingPulse/esp8266-oled-ssd1306, you may need the latest source and PR#198 for 64x48
#include <SSD1306Wire.h>
#define OLED_GEOMETRY     GEOMETRY_128_32
//#define OLED_GEOMETRY     GEOMETRY_64_48
//#define OLED_FLIP_V       1

//
// I2S Microphone reference values
//
// Datasheet: https://www.invensense.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.1.pdf
#define MIC_ICS43434      1  
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
//#define MIC_INMP441       1

#define MIC_OFFSET_DB     3.0103  // Default offset (sine-wave RMS vs. dBFS)
#define MIC_REF_DB        94.0    // dB(SPL)
#define MIC_BITS          24      // bits
#define MIC_REF_AMPL      420426  // Amplitude at 94dB(SPL) (-26dBFS from datasheet, i.e. (2^23−1)*10^(−26/20) )
#define MIC_OVERLOAD_DB   116.0   // dB(SPL) - Acoustic overload point
#define MIC_NOISE_DB      30      // dB(A) - Noise floor of the mic

//
// Sampling
//
#define SAMPLE_RATE       48000 // Hz
#define SAMPLE_BITS       32 // bits
#define SAMPLE_T          int32_t
#define SAMPLE_CONVERT(s) (s >> (SAMPLE_BITS - MIC_BITS)) 
#define SAMPLES_BLOCK     (SAMPLE_RATE / 8) // ~125ms 'fast' noise sampling
#define DMA_BANK_SIZE     (SAMPLES_BLOCK / 16)
#define DMA_BANKS         32

//
// I2S
// 
#define I2S_TASK_PRI      4
#define I2S_TASK_STACK    2048
#define I2S_PORT          I2S_NUM_0 

//NodeMCU-32S pin mapping (GPIO numbers)
#define I2S_WS            32
#define I2S_SCK           23 
#define I2S_SD            33 

//MH-ET ESP32 Mini pin mapping (GPIO numbers)
//#define I2S_WS            18 
//#define I2S_SCK           23 
//#define I2S_SD            19 


//
// IIR Filters
//
// Modified version of https://github.com/tttapa/Filters/blob/master/src/IIRFilter.h
// Replaced double with float, compacted and added GCC optimization / inlining
// Original code is licnsed under GPL-3.0
// 
class IIRFilter {
  public:  
    template <size_t B, size_t A>    
    IIRFilter(const float (&b)[B], const float (&_a)[A]) : lenB(B), lenA(A-1) {
      x = new float[lenB]();
      y = new float[lenA]();
      coeff_b = new float[2*lenB-1];
      coeff_a = new float[2*lenA-1];
      float a0 = _a[0];
      const float *a = &_a[1];
      for (uint8_t i = 0; i < 2*lenB-1; i++) coeff_b[i] = b[(2*lenB - 1 - i) % lenB] / a0;      
      for (uint8_t i = 0; i < 2*lenA-1; i++) coeff_a[i] = a[(2*lenA - 2 - i) % lenA] / a0;
    }
    
    ~IIRFilter() {
      delete[] x;
      delete[] y;
      delete[] coeff_a;
      delete[] coeff_b;
    }
    
    inline float filter(float value) __attribute__((optimize("-O3"))) {
      float b_terms = 0;
      x[i_b] = value;
      float *b_shift = &coeff_b[lenB - i_b - 1];
      for(uint8_t i = 0; i < lenB; i++) b_terms += x[i] * b_shift[i];      
      float a_terms = 0;
      float *a_shift = &coeff_a[lenA - i_a - 1];
      for(uint8_t i = 0; i < lenA; i++) a_terms += y[i] * a_shift[i];
      float filtered = b_terms - a_terms;
      y[i_a] = filtered;
      i_b++;
      if(i_b == lenB) i_b = 0;
      i_a++;
      if(i_a == lenA) i_a = 0;
      return filtered;
    }
    
  private:
    const uint8_t lenB, lenA;
    uint8_t i_b = 0, i_a = 0;
    float *x, *y, *coeff_b, *coeff_a;
};

// 
// Equalizer IIR filter to flatten microphone frequency response
// See respective .m file for filter design. Fs = 48Khz.
//
#if defined(MIC_ICS43434)
  const float Equalizer_B[] = {0.47733, -0.48649, -0.33646, 0.23462, 0.11102};
  const float Equalizer_A[] = {1.0, -1.9307338, 0.8651946, 0.0644284, 0.0011125};
#elif defined(MIC_INMP441)
  const float Equalizer_B[] = {1.00198, -1.99085, 0.98892};
  const float Equalizer_A[] = {1.0, -1.99518, 0.99518};
#endif
IIRFilter Equalizer(Equalizer_B, Equalizer_A);

//
// A-weighting 6th order IIR Filter, Fs = 48KHz 
// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
//
const float A_weighting_B[] = {0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003};
const float A_weighting_A[] = {1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968};
IIRFilter A_weighting(A_weighting_B, A_weighting_A);

//
// Golbals
//

// Data we push to 'samples_queue'
struct samples_sum_t {
  // Sum of squares of mic samples, data after Equalizer filter
  float sum_sqr_SPL;
  // Sum of squares of A-weighted mic samples
  float sum_sqr_A;
};

SSD1306Wire display(0x3c, SDA, SCL, OLED_GEOMETRY);
SAMPLE_T samples[SAMPLES_BLOCK];
QueueHandle_t samples_queue;

// Debug
#define DBG(...) printf(__VA_ARGS__) 
//#define DBG(...) {} // To disable

//
// I2S Setup
// 
void mic_i2s_init() {
  DBG("Setting up I2S for reading microphone...\n");

  // Setup I2S to sample mono channel for SAMPLE_RATE * SAMPLE_BITS
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BANKS,
    .dma_buf_len = DMA_BANK_SIZE,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  // I2S pin mapping
  const i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,  
    .ws_io_num    = I2S_WS,    
    .data_out_num = -1, // not used
    .data_in_num  = I2S_SD   
  };
  
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);    

  //FIXME: There is a known issue with esp-idf and sampling rates, see:
  //       https://github.com/espressif/esp-idf/issues/2634
  //       Update (when available) to the latest versions of esp-idf *should* fix it
  //       In the meantime, the below line seems to set sampling rate at ~47999.992Hz
  //       fifs_req=24576000, sdm0=149, sdm1=212, sdm2=5, odir=2 -> fifs_reached=24575996
  rtc_clk_apll_enable(1, 149, 212, 5, 2);    
}

//
// I2S Reader Task
//
// Rationale for separate task reading I2S is that IIR filter
// processing cam be scheduled to different core on the ESP32
// while i.e. the display is beeing updated...
//
void mic_i2s_reader_task(void* parameter) { 
  DBG("Starting noise reader task...\n");
  mic_i2s_init();

  // Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms)
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_BLOCK * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY); 
  
  while (true) {
    float sum_sqr_SPL = 0;
    float sum_sqr_A = 0;    
    SAMPLE_T sample;
    samples_sum_t ss;
    float s = 0;   

    i2s_read(I2S_PORT, &samples, SAMPLES_BLOCK * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);     
    for(int i=0; i<SAMPLES_BLOCK; i++) {
      sample = SAMPLE_CONVERT(samples[i]);  
      s = Equalizer.filter(sample);
      sum_sqr_SPL += (s * s);
      s = A_weighting.filter(s);
      sum_sqr_A += (s * s);
    }
    
    ss.sum_sqr_SPL = sum_sqr_SPL;
    ss.sum_sqr_A = sum_sqr_A;
    xQueueSend(samples_queue, &ss, portMAX_DELAY);
  }
}

void setup() {
  Serial.begin(112500);
  delay(1000);
  display.init();  
  #ifdef OLED_FLIP_V
    display.flipScreenVertically();
  #endif
    
  samples_queue = xQueueCreate(8, sizeof(samples_sum_t));
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);
  
  samples_sum_t ss;
  uint32_t slow_cnt = 0;
  float slow_sum_sqr = 0;
  float slow_dB_A = 0;

  // Read sum of samaples, calculated by 'i2s_reader_task'
  while (xQueueReceive(samples_queue, &ss, portMAX_DELAY)) {
    // Calculate dB values relative to MIC_REF_AMPL and adjust for reference dB
    float noise_dB_SPL = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(ss.sum_sqr_SPL / SAMPLES_BLOCK) / MIC_REF_AMPL);
    float fast_dB_A = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(ss.sum_sqr_A / SAMPLES_BLOCK) / MIC_REF_AMPL);
    slow_sum_sqr += ss.sum_sqr_A;
    slow_cnt += SAMPLES_BLOCK;
    if (slow_cnt >= SAMPLE_RATE) { // i.e. 1 second
      slow_dB_A = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(slow_sum_sqr / slow_cnt) / MIC_REF_AMPL);
      slow_sum_sqr = 0;
      slow_cnt = 0;
    }
    DBG("%.1f %.1f %.1f\n", noise_dB_SPL, fast_dB_A, slow_dB_A);
    
    display.clear();
    if (fast_dB_A > MIC_OVERLOAD_DB) {
      display.setFont(ArialMT_Plain_16);    
      display.drawString(0, 8, "Overload");
      display.display();
      continue;
    }
    // The 'fast' line
    uint16_t len = uint16_t(((fast_dB_A - MIC_NOISE_DB) / MIC_OVERLOAD_DB) * display.getWidth());
    display.drawHorizontalLine(0, 0, len);
    display.drawHorizontalLine(0, 1, len);
    display.drawHorizontalLine(0, 2, len);
    // The 'slow' numeric decibels
    display.setFont(ArialMT_Plain_24);    
    display.drawString(0, 6, String(slow_dB_A));
    // And units
    display.setFont(ArialMT_Plain_16);    
    if (display.getWidth() < 128) {
      display.drawString(24, 30, "dB(A)");
    } else {
      display.drawString(80, 14, "dB(A)");
    }    
    display.display();       
  }
}

void loop() {
  // Nothing here..
}  
