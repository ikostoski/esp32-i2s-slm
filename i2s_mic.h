/*
 * Display A-weighted sound level measured by I2S Microphone
 * 
 * (c)2019 Ivan Kostoski (original version)
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

#pragma once

#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "sos-iir-filter.h"

//#define SERIAL_OUTPUT

// I2S microphone connnection
// NR_OF_SAMPLES = number of microphone samples to take and return in queue
// I2S pins - Can be routed to almost any (unused) ESP32 pin.
//            SD can be any pin, including input only pins (36-39).
//            SCK (i.e. BCLK) and WS (i.e. L/R CLK) must be output capable pins
// PIN_WS = I2S word select pin
// PIN_SCK = I2S clock pin
// PIN_SD = I2S data pin
// I2S_PORT = I2S port to use
// MIC_BITS = number of valid bits in microphone data
// MSB_SHIFT = set to true to fix MSB timing for some microphones, i.e. SPH0645LM4H-x
// SAMPLE_RATE = microphone sample rate. must be 48kHz to fit filter design
template <unsigned NR_OF_SAMPLES, int PIN_WS = 18, int PIN_SCK = 23, int PIN_SD = 19, i2s_port_t I2S_PORT = I2S_NUM_0, unsigned MIC_BITS = 24, bool MSB_SHIFT = false, unsigned SAMPLE_RATE = 48000>
class Microphone_I2S
{
  static constexpr unsigned TASK_PRIO = 4;     // FreeRTOS priority
  static constexpr unsigned TASK_STACK = 2048; // FreeRTOS stack size (in 32-bit words)

public:
  using SAMPLE_T = int32_t;
  using SampleBuffer = float[NR_OF_SAMPLES];
  static const constexpr uint32_t SAMPLE_BITS = sizeof(SAMPLE_T) * 8;

  /// @brief Create new I2S microphone.
  /// @param filter Microphone IIR filter function to apply to samples
  Microphone_I2S(const SOS_IIR_Filter &filter)
      : m_filter(filter)
  {
  }

  void begin()
  {
#ifdef SERIAL_OUTPUT
    Serial.print("Installing microphone I2S driver at ");
    if (I2S_PORT == I2S_NUM_0)
    {
      Serial.println("I2S0");
    }
    else if (I2S_PORT == I2S_NUM_1)
    {
      Serial.println("I2S1");
    }
    else
    {
      Serial.println("unknown port");
    }
#endif
    // Setup I2S to sample mono channel for SAMPLE_RATE * SAMPLE_BITS
    // NOTE: Recent update to Arduino_esp32 (1.0.2 -> 1.0.3)
    //       seems to have swapped ONLY_LEFT and ONLY_RIGHT channels
    i2s_config_t i2s_config{};
    i2s_config.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.sample_rate = SAMPLE_RATE;
    i2s_config.bits_per_sample = i2s_bits_per_sample_t(SAMPLE_BITS);
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = 2;
    i2s_config.dma_buf_len = NR_OF_SAMPLES;
    i2s_config.use_apll = true;
    //i2s_config.tx_desc_auto_clear = false;
    i2s_config.fixed_mclk = 0;
    i2s_driver_install(I2S_PORT, &i2s_config, 0, nullptr);

    // I2S pin mapping
#ifdef SERIAL_OUTPUT
    Serial.println("Installing microphone I2S pin mapping");
#endif
    i2s_pin_config_t pin_config{};
    pin_config.bck_io_num = PIN_SCK;
    pin_config.ws_io_num = PIN_WS;
    pin_config.data_out_num = -1; // not used
    pin_config.data_in_num = PIN_SD;
    if (MSB_SHIFT)
    {
      // Undocumented (?!) manipulation of I2S peripheral registers
      // to fix MSB timing issues with some I2S microphones
      REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));
      REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);
    }
    i2s_set_pin(I2S_PORT, &pin_config);

#ifdef SERIAL_OUTPUT
    Serial.println("Creating microphone I2S sample queue");
#endif
    //FIXME: There is a known issue with esp-idf and sampling rates, see:
    //       https://github.com/espressif/esp-idf/issues/2634
    //       In the meantime, the below line seems to set sampling rate at ~47999.992Hz
    //       fifs_req=24576000, sdm0=149, sdm1=212, sdm2=5, odir=2 -> fifs_reached=24575996
    //NOTE:  This seems to be fixed in ESP32 Arduino 1.0.4, esp-idf 3.2
    //       Should be safe to remove...
    //#include <soc/rtc.h>
    //rtc_clk_apll_enable(1, 149, 212, 5, 2);
    // Create FreeRTOS queue
    m_sampleQueue = xQueueCreate(2, sizeof(SampleBuffer));
    // Create the I2S reader FreeRTOS task
    // NOTE: Current version of ESP-IDF will pin the task
    //       automatically to the first core it happens to run on
    //       (due to using the hardware FPU instructions).
    //       For manual control see: xTaskCreatePinnedToCore
#ifdef SERIAL_OUTPUT
    Serial.println("Creating microphone I2S reader task");
#endif
    xTaskCreate(readerTask, "Microphone_I2S reader", TASK_STACK, this, TASK_PRIO, nullptr);
  }

  /// @brief Get the queue that stores new sample buffers.
  QueueHandle_t sampleQueue() const
  {
    return m_sampleQueue;
  }

  /// @brief Start sampling from microphone.
  void startSampling()
  {
    m_isSampling = true;
  }

  /// @brief Stop sampling from microphone.
  void stopSampling()
  {
    m_isSampling = false;
  }

private:
  static void readerTask(void *parameter)
  {
#ifdef SERIAL_OUTPUT
    Serial.println("Mic reader task started");
#endif
    auto object = reinterpret_cast<Microphone_I2S *>(parameter);
    // Discard first blocks, microphone may have startup time (i.e. INMP441 up to 83ms)
    size_t bytes_read = 0;
    for (int i = 0; i < 5; i++)
    {
      i2s_read(I2S_PORT, &object->m_sampleBuffer, NR_OF_SAMPLES * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);
    }
    while (true)
    {
      if (object->m_isSampling)
      {
        // Block and wait for microphone values from I2S
        // Data is moved from DMA buffers to our m_sampleBuffer by the driver ISR
        // and when there is requested amount of data, task is unblocked
        //
        // Note: i2s_read does not care it is writing in float[] buffer, it will write
        //       integer values to the given address, as received from the hardware peripheral.
        i2s_read(I2S_PORT, &object->m_sampleBuffer, NR_OF_SAMPLES * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

        // Debug only. Ticks we spent filtering and summing block of I2S data
        //TickType_t start_tick = xTaskGetTickCount();

        // Convert (including shifting) integer microphone values to floats,
        // using the same buffer (assumed sample size is same as size of float),
        // to save a bit of memory
        auto int_samples = reinterpret_cast<const SAMPLE_T *>(&object->m_sampleBuffer);
        for (int i = 0; i < NR_OF_SAMPLES; i++)
        {
          object->m_sampleBuffer[i] = int_samples[i] >> (SAMPLE_BITS - MIC_BITS);
        }

        // filter values and apply gain setting
        object->m_filter.applyFilters(object->m_sampleBuffer, object->m_sampleBuffer, NR_OF_SAMPLES);
        object->m_filter.applyGain(object->m_sampleBuffer, object->m_sampleBuffer, NR_OF_SAMPLES);

        // Debug only. Ticks we spent filtering and summing block of I2S data
        //auto proc_ticks = xTaskGetTickCount() - start_tick;

        // Send the sums to FreeRTOS queue where main task will pick them up
        // and further calculate decibel values (division, logarithms, etc...)
        xQueueSend(object->m_sampleQueue, &object->m_sampleBuffer, portMAX_DELAY);

        // Debug only. Print raw microphone sample values
        /*int vMin = 1000000;
                int vMax = -vMin;
                int vAvg = 0;
                int vNan = 0;
                for (unsigned int k = 0; k < bytes_read; k++)
                {
                    if (isnan(object->m_sampleBuffer[k]) || isinf(object->m_sampleBuffer[k]))
                    {
                      object->m_sampleBuffer[k] = 0;
                      vNan++;
                    }
                    if (object->m_sampleBuffer[k] < vMin)
                    {
                      vMin = object->m_sampleBuffer[k];
                    }
                    if (object->m_sampleBuffer[k] > vMax)
                    {
                      vMax = object->m_sampleBuffer[k];
                    }
                    vAvg += object->m_sampleBuffer[k];
                }
                vAvg /= bytes_read;
                Serial.print("Min: "); Serial.print(vMin, 3);
                Serial.print(", Max: "); Serial.print(vMax, 3);
                Serial.print(", Avg: "); Serial.print(vAvg, 3);
                Serial.print(", NAN or INF: "); Serial.println(vNan);*/
      }
    }
  }

  SOS_IIR_Filter m_filter;
  QueueHandle_t m_sampleQueue;
  SampleBuffer m_sampleBuffer __attribute__((aligned(4)));
  bool m_isSampling = false;
};
