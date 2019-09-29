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
 * Displays line on the small OLED screen with 'short' LAeq(125ms)
 * response and numeric LAeq(1sec) dB value from the signal RMS.
 */

#include <driver/i2s.h>
#include <soc/rtc.h>
#include "iir-filter.h"

//
// Configuration
//

#define LEQ_PERIOD        1 // second(s)
#define WEIGHTING         A_weighting
#define LEQ_UNITS         "LAeq"
#define DB_UNITS          "dB(A)"
#define USE_DISPLAY       1

#define MIC_EQUALIZER     ICS43434  // See below for defined IIR filters
#define MIC_OFFSET_DB     3.0103    // Default offset (sine-wave RMS vs. dBFS)
#define MIC_REF_DB        94.0      // dB(SPL)
#define MIC_BITS          24        // valid bits in I2S data
#define MIC_REF_AMPL      420426    // Amplitude at 94dB(SPL) (-26dBFS from datasheet, i.e. (2^23-1)*10^(-26/20) )
#define MIC_OVERLOAD_DB   116.0     // dB - Acoustic overload point*/
#define MIC_NOISE_DB      30        // dB - Noise floor*/

//
// I2S pins
//
//NodeMCU-32S pin mapping (GPIO numbers)
#define I2S_WS            32
#define I2S_SCK           23
#define I2S_SD            33

//MH-ET ESP32 Mini pin mapping (GPIO numbers)
//#define I2S_WS            18 
//#define I2S_SCK           23 
//#define I2S_SD            19 

//
// Sampling
//
#define SAMPLE_RATE       48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS       32    // bits
#define SAMPLE_T          int32_t
#define SAMPLE_CONVERT(s) (s >> (SAMPLE_BITS - MIC_BITS))
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

//
// IIR Filters
//

// 
// Equalizer IIR filters to flatten microphone frequency response
// See respective .m file for filter design. Fs = 48Khz.
//

// TDK/InvenSense ICS-43434
// Datasheet: https://www.invensense.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.1.pdf
const double ICS43434_B[] = {0.47733, -0.48649, -0.33646, 0.23462, 0.11102};
const double ICS43434_A[] = {1.0, -1.9307338, 0.8651946, 0.0644284, 0.0011125};
IIR_FILTER ICS43434(ICS43434_B, ICS43434_A);

// TDK/InvenSense ICS-43432
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/ICS-43432-data-sheet-v1.3.pdf
const double ICS43432_B[] = {-0.457337, 1.1222867, -0.77818279, 0.0096893, 0.1034567};
const double ICS43432_A[] = {1,-3.3420781, 4.4033694, -3.0167073, 1.2265537, -0.2962229, 0.0251086};
IIR_FILTER ICS43432(ICS43432_B, ICS43432_A);

// TDK/InvenSense INMP441
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
const double INMP441_B[] = {1.00198, -1.99085, 0.98892};
const double INMP441_A[] = {1.0, -1.99518, 0.99518};
IIR_FILTER INMP441(INMP441_B, INMP441_A);

//
// A-weighting 6th order IIR Filter, Fs = 48KHz 
// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
//
const double A_weighting_B[] = {0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003};
const double A_weighting_A[] = {1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968};
IIR_FILTER A_weighting(A_weighting_B, A_weighting_A);

// No filter, class to disable equalizer or weighting
class NoFilter {
  public:
    inline IIR_BASE_T filter(IIR_BASE_T value) { return value; };
};
NoFilter None;

#if (USE_DISPLAY > 0)
  // ThingPulse/esp8266-oled-ssd1306, you may need the latest source and PR#198 for 64x48
  #include <SSD1306Wire.h>
  #define OLED_GEOMETRY GEOMETRY_128_32
  //#define OLED_GEOMETRY     GEOMETRY_64_48
  //#define OLED_FLIP_V       1
  SSD1306Wire display(0x3c, SDA, SCL, OLED_GEOMETRY);
#endif

// Data we push to 'samples_queue'
struct samples_queue_t {
  // Sum of squares of mic samples, after Equalizer filter
  IIR_ACCU_T sum_sqr_SPL;
  // Sum of squares of weighted mic samples
  IIR_ACCU_T sum_sqr_weighted;
};
int32_t samples[SAMPLES_SHORT];
QueueHandle_t samples_queue;

//
// I2S Setup
// 
#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048
#define I2S_PORT       I2S_NUM_0

void mic_i2s_init() {
  // Setup I2S to sample mono channel for SAMPLE_RATE * SAMPLE_BITS
  // NOTE: Recent update to Arduino_esp32 (1.0.2 -> 1.0.3)
  //       seems to have swapped ONLY_LEFT and ONLY_RIGHT channels
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
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
  mic_i2s_init();

  // Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms)
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    IIR_ACCU_T sum_sqr_SPL = 0;
    IIR_ACCU_T sum_sqr_weighted = 0;
    samples_queue_t q;
    int32_t sample;
    IIR_BASE_T s;

    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);
    for(int i=0; i<SAMPLES_SHORT; i++) {
      sample = SAMPLE_CONVERT(samples[i]);
      s = sample;
      s = MIC_EQUALIZER.filter(s);
      ACCU_SQR(sum_sqr_SPL, s);
      s = WEIGHTING.filter(s);
      ACCU_SQR(sum_sqr_weighted, s);
    }

    q.sum_sqr_SPL = sum_sqr_SPL;
    q.sum_sqr_weighted = sum_sqr_weighted;
    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

void setup() {
  Serial.begin(112500);
  delay(1000);
  #if (USE_DISPLAY > 0)
    display.init();
    #if (OLED_FLIP_V > 0)
      display.flipScreenVertically();
    #endif
  #endif

  samples_queue = xQueueCreate(8, sizeof(samples_queue_t));
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);

  samples_queue_t q;
  uint32_t Leq_cnt = 0;
  IIR_ACCU_T Leq_sum_sqr = 0;
  double Leq_dB = 0;

  // Read sum of samaples, calculated by 'i2s_reader_task'
  while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {

    // Calculate dB values relative to MIC_REF_AMPL and adjust for reference dB
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT) / MIC_REF_AMPL);

    // In case of acoustic overload, report infinty Leq value
    if (short_SPL_dB > MIC_OVERLOAD_DB) Leq_sum_sqr = INFINITY;

    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_cnt += SAMPLES_SHORT;
    if (Leq_cnt >= SAMPLE_RATE * LEQ_PERIOD) {
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(double(Leq_sum_sqr) / Leq_cnt) / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_cnt = 0;
      printf("%s(%ds) : %.1f\n", LEQ_UNITS, LEQ_PERIOD, Leq_dB);
    }

    #if (USE_DISPLAY > 0)
      display.clear();
      if (short_SPL_dB > MIC_OVERLOAD_DB) {
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 8, "Overload");
        display.display();
        continue;
      }
      // The 'short' Leq line
      double short_Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(double(q.sum_sqr_weighted) / SAMPLES_SHORT) / MIC_REF_AMPL);
      uint16_t len = uint16_t(((short_Leq_dB - MIC_NOISE_DB) / MIC_OVERLOAD_DB) * display.getWidth());
      display.drawHorizontalLine(0, 0, len);
      display.drawHorizontalLine(0, 1, len);
      display.drawHorizontalLine(0, 2, len);
      // The Leq numeric decibels
      display.setFont(ArialMT_Plain_24);
      display.drawString(0, 6, String(Leq_dB, 1));
      // And units
      display.setFont(ArialMT_Plain_16);
      if (display.getWidth() < 128) {
        display.drawString(24, 30, DB_UNITS);
      } else {
        display.drawString(80, 14, DB_UNITS);
      }
      display.display();
    #endif // USE_DISPLAY
  }
}

void loop() {
  // Nothing here..
}
