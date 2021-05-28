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

#include "sos-iir-filter.h"

//
// IIR Filters
//

// DC-Blocker filter - removes DC component from I2S data
// See: https://www.dsprelated.com/freebooks/filters/DC_Blocker.html
// a1 = -0.9992 should heavily attenuate frequencies below 10Hz
SOS_IIR_Filter DC_BLOCKER = {
  gain : 1.0,
  sos : {{-1.0, 0.0, +0.9992, 0}}
};

//
// Equalizer IIR filters to flatten microphone frequency response
// See respective .m file for filter design. Fs = 48Khz.
//
// Filters are represented as Second-Order Sections cascade with assumption
// that b0 and a0 are equal to 1.0 and 'gain' is applied at the last step
// B and A coefficients were transformed with GNU Octave:
// [sos, gain] = tf2sos(B, A)
// See: https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
// NOTE: SOS matrix 'a1' and 'a2' coefficients are negatives of tf2sos output
//

// TDK/InvenSense ICS-43434
// Datasheet: https://www.invensense.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.1.pdf
// B = [0.477326418836803, -0.486486982406126, -0.336455844522277, 0.234624646917202, 0.111023257388606];
// A = [1.0, -1.93073383849136326, 0.86519456089576796, 0.06442838283825100, 0.00111249298800616];
SOS_IIR_Filter ICS43434 = {
  gain : 0.477326418836803,
  sos : {// Second-Order Sections {b1, b2, -a1, -a2}
         {+0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128},
         {-1.98905931743624453, 0.98908924206960169, +1.99755331853906037, -0.99755481510122113}}
};

// TDK/InvenSense ICS-43432
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/ICS-43432-data-sheet-v1.3.pdf
// B = [-0.45733702338341309   1.12228667105574775  -0.77818278904413563, 0.00968926337978037, 0.10345668405223755]
// A = [1.0, -3.3420781082912949, 4.4033694320978771, -3.0167072679918010, 1.2265536567647031, -0.2962229189311990, 0.0251085747458112]
SOS_IIR_Filter ICS43432 = {
  gain : -0.457337023383413,
  sos : {// Second-Order Sections {b1, b2, -a1, -a2}
         {-0.544047931916859, -0.248361759321800, +0.403298891662298, -0.207346186351843},
         {-1.909911869441421, +0.910830292683527, +1.790285722826743, -0.804085812369134},
         {+0.000000000000000, +0.000000000000000, +1.148493493802252, -0.150599527756651}}
};

// TDK/InvenSense INMP441
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
// B ~= [1.00198, -1.99085, 0.98892]
// A ~= [1.0, -1.99518, 0.99518]
SOS_IIR_Filter INMP441 = {
  gain : 1.00197834654696,
  sos : {// Second-Order Sections {b1, b2, -a1, -a2}
         {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}}
};

// Infineon IM69D130 Shield2Go
// Datasheet: https://www.infineon.com/dgdl/Infineon-IM69D130-DS-v01_00-EN.pdf?fileId=5546d462602a9dc801607a0e46511a2e
// B ~= [1.001240684967527, -1.996936108836337, 0.995703101823006]
// A ~= [1.0, -1.997675693595542, 0.997677044195563]
// With additional DC blocking component
SOS_IIR_Filter IM69D130 = {
  gain : 1.00124068496753,
  sos : {
      {-1.0, 0.0, +0.9992, 0}, // DC blocker, a1 = -0.9992
      {-1.994461610298131, 0.994469278738208, +1.997675693595542, -0.997677044195563}}
};

// Knowles SPH0645LM4H-B, rev. B
// https://cdn-shop.adafruit.com/product-files/3421/i2S+Datasheet.PDF
// B ~= [1.001234, -1.991352, 0.990149]
// A ~= [1.0, -1.993853, 0.993863]
// With additional DC blocking component
SOS_IIR_Filter SPH0645LM4H_B_RB = {
  gain : 1.00123377961525,
  sos : {                         // Second-Order Sections {b1, b2, -a1, -a2}
         {-1.0, 0.0, +0.9992, 0}, // DC blocker, a1 = -0.9992
         {-1.988897663539382, +0.988928479008099, +1.993853376183491, -0.993862821429572}}
};

//
// Weighting filters
//

//
// A-weighting IIR Filter, Fs = 48KHz
// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
// B = [0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003]
// A = [1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968]
SOS_IIR_Filter A_weighting = {
  gain : 0.169994948147430,
  sos : {// Second-Order Sections {b1, b2, -a1, -a2}
         {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
         {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
         {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}}
};

//
// C-weighting IIR Filter, Fs = 48KHz
// Designed by invfreqz curve-fitting, see respective .m file
// B = [-0.49164716933714026, 0.14844753846498662, 0.74117815661529129, -0.03281878334039314, -0.29709276192593875, -0.06442545322197900, -0.00364152725482682]
// A = [1.0, -1.0325358998928318, -0.9524000181023488, 0.8936404694728326   0.2256286147169398  -0.1499917107550188, 0.0156718181681081]
SOS_IIR_Filter C_weighting = {
  gain : -0.491647169337140,
  sos : {
      {+1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
      {+0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
      {-2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}}
};
