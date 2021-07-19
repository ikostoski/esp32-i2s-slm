/*
 * ESP32 Second-Order Sections IIR Filter implementation
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

#pragma once

#include <cstdint>
#include <initializer_list>
#include <vector>

struct SOS_Coefficients
{
    float b1;
    float b2;
    float a1;
    float a2;
};

struct SOS_Delay_State
{
    float w0 = 0.0F;
    float w1 = 0.0F;
};

extern "C"
{
    int sos_filter_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w);
}
__asm__(
    //
    // ESP32 implementation of IIR Second-Order Section filter
    // Assumes a0 and b0 coefficients are one (1.0)
    //
    // float* a2 = input;
    // float* a3 = output;
    // int    a4 = len;
    // float* a5 = coeffs;
    // float* a6 = w;
    // float  a7 = gain;
    //
    ".text                    \n"
    ".align  4                \n"
    ".global sos_filter_f32   \n"
    ".type   sos_filter_f32,@function\n"
    "sos_filter_f32:          \n"
    "  entry   a1, 16         \n"
    "  lsi     f0, a5, 0      \n" // float f0 = coeffs.b1;
    "  lsi     f1, a5, 4      \n" // float f1 = coeffs.b2;
    "  lsi     f2, a5, 8      \n" // float f2 = coeffs.a1;
    "  lsi     f3, a5, 12     \n" // float f3 = coeffs.a2;
    "  lsi     f4, a6, 0      \n" // float f4 = w[0];
    "  lsi     f5, a6, 4      \n" // float f5 = w[1];
    "  loopnez a4, 1f         \n" // for (; len>0; len--) {
    "    lsip    f6, a2, 4    \n" //   float f6 = *input++;
    "    madd.s  f6, f2, f4   \n" //   f6 += f2 * f4; // coeffs.a1 * w0
    "    madd.s  f6, f3, f5   \n" //   f6 += f3 * f5; // coeffs.a2 * w1
    "    mov.s   f7, f6       \n" //   f7 = f6; // b0 assumed 1.0
    "    madd.s  f7, f0, f4   \n" //   f7 += f0 * f4; // coeffs.b1 * w0
    "    madd.s  f7, f1, f5   \n" //   f7 += f1 * f5; // coeffs.b2 * w1 -> result
    "    ssip    f7, a3, 4    \n" //   *output++ = f7;
    "    mov.s   f5, f4       \n" //   f5 = f4; // w1 = w0
    "    mov.s   f4, f6       \n" //   f4 = f6; // w0 = f6
    "  1:                     \n" // }
    "  ssi     f4, a6, 0      \n" // w[0] = f4;
    "  ssi     f5, a6, 4      \n" // w[1] = f5;
    "  movi.n   a2, 0         \n" // return 0;
    "  retw.n                 \n");

/// @brief Envelops above asm function into C++ class
class SOS_IIR_Filter
{
public:
    /// @brief Constructor
    SOS_IIR_Filter(float gain, const std::vector<SOS_Coefficients>& sos)
        : m_gain(gain)
        , m_sos(sos)
        , m_w(sos.size())
    {
    };

    /// @brief Apply defined IIR Filter(s) to input array of floats and write values to output
    auto applyFilters(float *input, float *output, size_t len) const -> void
    {
        float *source = input;
        for (decltype(m_sos.size()) i = 0; i < m_sos.size(); i++)
        {
            sos_filter_f32(source, output, len, m_sos[i], m_w[i]);
            source = output;
        }
    }

    /// @brief Apply filter gain to input data and write values to output
    auto applyGain(const float *input, float *output, size_t len) const -> void
    {
        if (input != nullptr && output != nullptr)
        {
            for (decltype(len) i = 0; i < len; i++)
            {
              output[i] = input[i] * m_gain;
            }
        }
    }

    /// @brief Calculate sum of squares of all values in input array
    auto calculateSumOfSquares(const float *input, size_t len) const -> float
    {
        float result = 0.0F;
        if (input != nullptr)
        {
            for (decltype(len) i = 0; i < len; i++)
            {
              result += input[i] * input[i];
            }
        }
        return result;
    }
    
private:
    float m_gain = 1.0F;
    std::vector<SOS_Coefficients> m_sos{};
    mutable std::vector<SOS_Delay_State> m_w{};
};
