/*
 * Modified version of 
 *   https://github.com/tttapa/Filters/blob/master/src/IIRFilter.h
 * Part of:
 *   Arduino finite impulse response and infinite impulse response filter library 
 *   https://github.com/tttapa/Filters
 *   
 * Original code is licensed under GPL-3.0
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

#include <stdint.h>

/******************************************************************************/

//
// Fixed point .32 precision (integer math)
// - On ESP32, this seems to be ~2x faster than (emulated) double-precision
//   and should be good enough for 2 x 6th order IIR filters
// - Error (difference to double precision) is <= 0.1dB down to 10Hz
// - Should not overflow with 24bit microphone values (120dBFS)
//
#define IIR_ACCU_T         fixed_point_64_32
#define IIR_BASE_T         fixed_point_32_32
#define ACCU_MUL(A, X, Y)  A.accu_mul(X, Y)
#define ACCU_SQR(A, X)     A.accu_sqr(X)

//
// Double precision floating point math
// - Works with up to 4th + 6th order IIR filters
// - With 2 x 6th order filters, usees up 122ms from 125ms available
//   on signle core.
// - Ask Espressif nicely to enable, integrate or otherwise publish the
//   'double-precission accelerator instructions' with appropriate libraries
//   (silicon should be in the ESP32 cores), to get ~2x improvement in DP-FPU speed.
//
//#define IIR_ACCU_T         double
//#define IIR_BASE_T         double
//#define ACCU_MUL(A, X, Y)  A += (X) * (Y)
//#define ACCU_SQR(A, X)     A += (X) * (X)

// NOTE: IIR filters with single-precision floats may
//       produce errors (>19db) for low frequencies below 40Hz
//       (i.e. in A-weighting filter)

/******************************************************************************/

#define IIR_FILTER         IIRFilter<IIR_ACCU_T, IIR_BASE_T>

/******************************************************************************
 * Fixed point 64.32 and 32.32
 * - 32bit platforms don't get __int128 in GCC
 * - Minimalistic implementation (only what is needed for IIRFilter)
 ******************************************************************************/

#define INLINE            __attribute__((optimize("-O3"), always_inline)) inline
#define high_word(x)      (x >> 32)
#define low_word(x)       (uint32_t)x

struct fixed_point_32_32;

struct fixed_point_64_32 {
  int64_t v;
  uint32_t f;

  fixed_point_64_32(): v(0), f(0) {};
  fixed_point_64_32(double d) { v = d; int64_t r = d * 0x100000000LL; f = low_word(r); };
  explicit operator double() const { return (1.0 * f / 0x100000000LL) + v; }

  fixed_point_64_32& operator += (const fixed_point_64_32& a);
  void accu_mul(const fixed_point_32_32 a, const fixed_point_32_32 b);
  void accu_sqr(const fixed_point_32_32 a);
};

struct fixed_point_32_32 {
  int32_t v;
  uint32_t f;
  fixed_point_32_32(): v(0), f(0) {};
  fixed_point_32_32(int32_t i): v(i), f(0) {};
  fixed_point_32_32(long long i) { v = high_word(i); f = low_word(i); };
  fixed_point_32_32(long long unsigned i) { v = high_word(i); f = low_word(i); };
  fixed_point_32_32(double d) { int64_t i = d * 0x100000000LL; v = high_word(i); f = low_word(i); };
  fixed_point_32_32(fixed_point_64_32 fp) { v = fp.v; f = fp.f; };
  explicit operator int64_t() const { return (int64_t(v) << 32) + f; };
  explicit operator double() const { return (1.0 * f / 0x100000000LL) + v; }
};

// Assume we have MUL32_HIGH on ESP32
#define mul32x32uu(A, B)  ((uint64_t)(A) * (B))
#define mul32x32su(A, B)  ((int64_t)(A) * (B))
#define mul32x32ss(A, B)  ((int64_t)(A) * (B))

/*
 * Fixed point 64.32 - Add a
 */
INLINE fixed_point_64_32& fixed_point_64_32::operator += (const fixed_point_64_32& a) {
  uint64_t sumf = f + a.f;
  f = low_word(sumf);
  v += a.v + high_word(sumf);
  return *this;
}

/*
 * Fixed point 64.32 - Multiply a * b and accumulatee
 */
INLINE void fixed_point_64_32::accu_mul(const fixed_point_32_32 a, const fixed_point_32_32 b) {
  int64_t value = mul32x32ss(a.v, b.v);
  uint64_t fract = high_word(mul32x32uu(a.f, b.f)); // Discarding 32 LSB bits here
  int64_t p = mul32x32su(a.v, b.f); // a.v * b.f
  fract += low_word(p);
  value += high_word(p);
  p = mul32x32su(b.v, a.f) ; // b.v * a.f
  fract += low_word(p);
  value += high_word(p);
  fract += f; // accumulate with carry
  f = low_word(fract);
  v += value + high_word(fract);
}

/*
 * Fixed point 64.32, trades one 32x32 multiplication for two shifts
 */
INLINE void fixed_point_64_32::accu_sqr(const fixed_point_32_32 a) {
  int64_t value = mul32x32ss(a.v, a.v);
  uint64_t fract = high_word(mul32x32uu(a.f, a.f)); // Discarding 32 LSB bits here
  int64_t p = mul32x32su(a.v, a.f); // a.v * a.f
  fract += (low_word(p) << 1);
  value += (high_word(p) << 1);
  fract += f;
  f = low_word(fract);
  v += value + high_word(fract);
}

/******************************************************************************
 * IIR Filter implementation, direct form I, template
 ******************************************************************************/
template<typename AccuT, typename CoeffT>
class IIRFilter {
  public:  
    template <size_t B, size_t A>    
    IIRFilter(const double (&b)[B], const double (&_a)[A]) : lenB(B), lenA(A-1) {
      x = new CoeffT[lenB]();
      y = new CoeffT[lenA]();
      coeff_b = new CoeffT[2*lenB-1];
      coeff_a = new CoeffT[2*lenA-1];
      double a0 = -1 / _a[0];
      const double *a = &_a[1];
      for (uint8_t i = 0; i < 2*lenB-1; i++) coeff_b[i] = b[(2*lenB - 1 - i) % lenB] * a0;
      for (uint8_t i = 0; i < 2*lenA-1; i++) coeff_a[i] = a[(2*lenA - 2 - i) % lenA] * a0;
    }
    
    ~IIRFilter() {
      delete[] x;
      delete[] y;
      delete[] coeff_a;
      delete[] coeff_b;
    }

    INLINE CoeffT filter(CoeffT value) {
      AccuT filtered = 0;
      x[i_b] = value;
      CoeffT *b_shift = &coeff_b[lenB - i_b - 1];
      for(uint8_t i = 0; i < lenB; i++) ACCU_MUL(filtered, x[i], b_shift[i]);
      CoeffT *a_shift = &coeff_a[lenA - i_a - 1];
      for(uint8_t i = 0; i < lenA; i++) ACCU_MUL(filtered, y[i], a_shift[i]);
      y[i_a] = filtered;
      i_b++;
      if(i_b == lenB) i_b = 0;
      i_a++;
      if(i_a == lenA) i_a = 0;
      return filtered;
    }
    
  protected:
    const uint8_t lenB, lenA;
    uint8_t i_b = 0, i_a = 0;
    CoeffT *x, *y, *coeff_b, *coeff_a;
};
