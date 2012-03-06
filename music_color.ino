/**
 * This is the Color Organ / Spectrum Analyzer from
 * http://fftarduino.blogspot.com/2011/02/color-organ-spectrum-analyzer-on.html
 *
 * It's been modified to work with the Spark Fun WS2801 LED strip.
 */

#include <avr/pgmspace.h>
#include "SPI.h"
#include "WS2801.h"

const int ledCountKr = 11;    // the number of levels on the display for red color band
const int ledCountZl = 11;    // the number of levels on the display for green color band
const int ledCountSn = 10;    // the number of levels on the display for blue color band

WS2801 strip = WS2801(ledCountKr + ledCountZl + ledCountSn);

/* fix_fft.c - Fixed-point in-place Fast Fourier Transform  */
/*
 All data are fixed-point short integers, in which -32768
 to +32768 represent -1.0 to +1.0 respectively. Integer
 arithmetic is used for speed, instead of the more natural
 floating-point.

 Written by:  Tom Roberts  11/8/89
 Made portable:  Malcolm Slaney 12/15/94 malcolm@interval.com
 Enhanced:  Dimitrios P. Bouras  14 Jun 2006 dbouras@ieee.org
 Adapted for Arduino: Anatoly Kuzmenko 6 Feb 2011 k_anatoly@hotmail.com
*/

//**************************************************************************************************
#define N_WAVE          1024    /* full length of Sinewave[] */
#define LOG2_N_WAVE     10      /* log2(N_WAVE) */

#define FFT_SIZE       64 
#define log2FFT       6
#define N             (2 * FFT_SIZE)
#define log2N         (log2FFT + 1)

const prog_int16_t Sinewave[N_WAVE-N_WAVE/4] PROGMEM = {
  0,    201,    402,    603,    804,   1005,   1206,   1406,
  1607,   1808,   2009,   2209,   2410,   2610,   2811,   3011,
  3211,   3411,   3611,   3811,   4011,   4210,   4409,   4608,
  4807,   5006,   5205,   5403,   5601,   5799,   5997,   6195,
  6392,   6589,   6786,   6982,   7179,   7375,   7571,   7766,
  7961,   8156,   8351,   8545,   8739,   8932,   9126,   9319,
  9511,   9703,   9895,  10087,  10278,  10469,  10659,  10849,
  11038,  11227,  11416,  11604,  11792,  11980,  12166,  12353,
  12539,  12724,  12909,  13094,  13278,  13462,  13645,  13827,
  14009,  14191,  14372,  14552,  14732,  14911,  15090,  15268,
  15446,  15623,  15799,  15975,  16150,  16325,  16499,  16672,
  16845,  17017,  17189,  17360,  17530,  17699,  17868,  18036,
  18204,  18371,  18537,  18702,  18867,  19031,  19194,  19357,
  19519,  19680,  19840,  20000,  20159,  20317,  20474,  20631,
  20787,  20942,  21096,  21249,  21402,  21554,  21705,  21855,
  22004,  22153,  22301,  22448,  22594,  22739,  22883,  23027,
  23169,  23311,  23452,  23592,  23731,  23869,  24006,  24143,
  24278,  24413,  24546,  24679,  24811,  24942,  25072,  25201,
  25329,  25456,  25582,  25707,  25831,  25954,  26077,  26198,
  26318,  26437,  26556,  26673,  26789,  26905,  27019,  27132,
  27244,  27355,  27466,  27575,  27683,  27790,  27896,  28001,
  28105,  28208,  28309,  28410,  28510,  28608,  28706,  28802,
  28897,  28992,  29085,  29177,  29268,  29358,  29446,  29534,
  29621,  29706,  29790,  29873,  29955,  30036,  30116,  30195,
  30272,  30349,  30424,  30498,  30571,  30643,  30713,  30783,
  30851,  30918,  30984,  31049,  31113,  31175,  31236,  31297,
  31356,  31413,  31470,  31525,  31580,  31633,  31684,  31735,
  31785,  31833,  31880,  31926,  31970,  32014,  32056,  32097,
  32137,  32176,  32213,  32249,  32284,  32318,  32350,  32382,
  32412,  32441,  32468,  32495,  32520,  32544,  32567,  32588,
  32609,  32628,  32646,  32662,  32678,  32692,  32705,  32717,
  32727,  32736,  32744,  32751,  32757,  32761,  32764,  32766,
  32767,  32766,  32764,  32761,  32757,  32751,  32744,  32736,
  32727,  32717,  32705,  32692,  32678,  32662,  32646,  32628,
  32609,  32588,  32567,  32544,  32520,  32495,  32468,  32441,
  32412,  32382,  32350,  32318,  32284,  32249,  32213,  32176,
  32137,  32097,  32056,  32014,  31970,  31926,  31880,  31833,
  31785,  31735,  31684,  31633,  31580,  31525,  31470,  31413,
  31356,  31297,  31236,  31175,  31113,  31049,  30984,  30918,
  30851,  30783,  30713,  30643,  30571,  30498,  30424,  30349,
  30272,  30195,  30116,  30036,  29955,  29873,  29790,  29706,
  29621,  29534,  29446,  29358,  29268,  29177,  29085,  28992,
  28897,  28802,  28706,  28608,  28510,  28410,  28309,  28208,
  28105,  28001,  27896,  27790,  27683,  27575,  27466,  27355,
  27244,  27132,  27019,  26905,  26789,  26673,  26556,  26437,
  26318,  26198,  26077,  25954,  25831,  25707,  25582,  25456,
  25329,  25201,  25072,  24942,  24811,  24679,  24546,  24413,
  24278,  24143,  24006,  23869,  23731,  23592,  23452,  23311,
  23169,  23027,  22883,  22739,  22594,  22448,  22301,  22153,
  22004,  21855,  21705,  21554,  21402,  21249,  21096,  20942,
  20787,  20631,  20474,  20317,  20159,  20000,  19840,  19680,
  19519,  19357,  19194,  19031,  18867,  18702,  18537,  18371,
  18204,  18036,  17868,  17699,  17530,  17360,  17189,  17017,
  16845,  16672,  16499,  16325,  16150,  15975,  15799,  15623,
  15446,  15268,  15090,  14911,  14732,  14552,  14372,  14191,
  14009,  13827,  13645,  13462,  13278,  13094,  12909,  12724,
  12539,  12353,  12166,  11980,  11792,  11604,  11416,  11227,
  11038,  10849,  10659,  10469,  10278,  10087,   9895,   9703,
  9511,   9319,   9126,   8932,   8739,   8545,   8351,   8156,
  7961,   7766,   7571,   7375,   7179,   6982,   6786,   6589,
  6392,   6195,   5997,   5799,   5601,   5403,   5205,   5006,
  4807,   4608,   4409,   4210,   4011,   3811,   3611,   3411,
  3211,   3011,   2811,   2610,   2410,   2209,   2009,   1808,
  1607,   1406,   1206,   1005,    804,    603,    402,    201,
  0,   -201,   -402,   -603,   -804,  -1005,  -1206,  -1406,
  -1607,  -1808,  -2009,  -2209,  -2410,  -2610,  -2811,  -3011,
  -3211,  -3411,  -3611,  -3811,  -4011,  -4210,  -4409,  -4608,
  -4807,  -5006,  -5205,  -5403,  -5601,  -5799,  -5997,  -6195,
  -6392,  -6589,  -6786,  -6982,  -7179,  -7375,  -7571,  -7766,
  -7961,  -8156,  -8351,  -8545,  -8739,  -8932,  -9126,  -9319,
  -9511,  -9703,  -9895, -10087, -10278, -10469, -10659, -10849,
  -11038, -11227, -11416, -11604, -11792, -11980, -12166, -12353,
  -12539, -12724, -12909, -13094, -13278, -13462, -13645, -13827,
  -14009, -14191, -14372, -14552, -14732, -14911, -15090, -15268,
  -15446, -15623, -15799, -15975, -16150, -16325, -16499, -16672,
  -16845, -17017, -17189, -17360, -17530, -17699, -17868, -18036,
  -18204, -18371, -18537, -18702, -18867, -19031, -19194, -19357,
  -19519, -19680, -19840, -20000, -20159, -20317, -20474, -20631,
  -20787, -20942, -21096, -21249, -21402, -21554, -21705, -21855,
  -22004, -22153, -22301, -22448, -22594, -22739, -22883, -23027,
  -23169, -23311, -23452, -23592, -23731, -23869, -24006, -24143,
  -24278, -24413, -24546, -24679, -24811, -24942, -25072, -25201,
  -25329, -25456, -25582, -25707, -25831, -25954, -26077, -26198,
  -26318, -26437, -26556, -26673, -26789, -26905, -27019, -27132,
  -27244, -27355, -27466, -27575, -27683, -27790, -27896, -28001,
  -28105, -28208, -28309, -28410, -28510, -28608, -28706, -28802,
  -28897, -28992, -29085, -29177, -29268, -29358, -29446, -29534,
  -29621, -29706, -29790, -29873, -29955, -30036, -30116, -30195,
  -30272, -30349, -30424, -30498, -30571, -30643, -30713, -30783,
  -30851, -30918, -30984, -31049, -31113, -31175, -31236, -31297,
  -31356, -31413, -31470, -31525, -31580, -31633, -31684, -31735,
  -31785, -31833, -31880, -31926, -31970, -32014, -32056, -32097,
  -32137, -32176, -32213, -32249, -32284, -32318, -32350, -32382,
  -32412, -32441, -32468, -32495, -32520, -32544, -32567, -32588,
  -32609, -32628, -32646, -32662, -32678, -32692, -32705, -32717,
  -32727, -32736, -32744, -32751, -32757, -32761, -32764, -32766
};

int fix_fft(int fr[], int fi[], int m )
{
  int mr, nn, i, j, l, k, istep, n, scale, shift;
  int qr, qi, tr, ti, wr, wi;

  n = 1 << m;

  /* max FFT size = N_WAVE */
  if (n > N_WAVE) {
    return -1;
  }
  
  mr = 0;
  nn = n - 1;
  scale = 0;

  /* decimation in time - re-order data */
  for (m=1; m<=nn; ++m) {
    l = n;
    do {
      l >>= 1;
    } 
    while (mr+l > nn);
    
    mr = (mr & (l-1)) + l;
    
    if (mr <= m) {
       continue;
    }
    tr = fr[m];
    fr[m] = fr[mr];
    fr[mr] = tr;
    ti = fi[m];
    fi[m] = fi[mr];
    fi[mr] = ti;
  }

  l = 1;
  k = LOG2_N_WAVE-1;
  while (l < n) {
    shift = 1;
    istep = l << 1;
    for (m=0; m<l; ++m) {
      j = m << k;
      /* 0 <= j < N_WAVE/2 */
      wr =  pgm_read_word(&Sinewave[j+N_WAVE/4]);
      wi = -pgm_read_word(&Sinewave[j]);

      wr >>= 1;
      wi >>= 1;

      for (i=m; i<n; i+=istep) {
        j = i + l;
        tr = ((long)wr*(long)fr[j] - (long)wi*(long)fi[j])>>15;
        ti = ((long)wr*(long)fi[j] + (long)wi*(long)fr[j])>>15;
        qr = fr[i];
        qi = fi[i];

        qr >>= 1;
        qi >>= 1;

        fr[j] = qr - tr;
        fi[j] = qi - ti;
        fr[i] = qr + tr;
        fi[i] = qi + ti;
      }
    }
    --k;
    l = istep;
  }
  return scale;
}

int fix_fftr(int f[], int m )
{
  int i, Nt = 1<<(m-1),  scale = 0;
  int tt, *fr=f, *fi=&f[Nt];

  scale = fix_fft(fi, fr, m-1 );
  return scale;
}
//**************************************************************************************************

int       x[N], fx[N];
int      incomingByte; 
int   i, count, scale;
int kraccums = 0, zlaccums = 0, snaccums = 0;
int kraccumn = 0, zlaccumn = 0, snaccumn = 0;

int sdvig = 32768; //DC bias of the ADC, approxim +2.5V. (kompensaciya post. sostavlyauschei).
int minim = 0; 
int maxim = 32000; 
int vrem;
float kdmp = 0.95;  //Smoothing constant. 
float kary = 0.999;  //AGC time constant. 
                      //AGC affects visual display only, no AGC on analog part of the system


void setup() {

  // initialize serial communication, for debug purposes mostly,
  //be careful with serial on Linux (Ubuntu), it hangs up periodicaly:
  Serial.begin(115200);  
  strip.begin();
  strip.show();
   
}

void loop()
{
//Filling up input raw data array x[];
// 14.6 msec for ADC and 12.4 msec everything else, TOTAL = 27 msec if FFT size = 64 (N=128).
// 28.8 msec for ADC and 27.1 msec everything else, TOTAL = 55.9 msec if FFT size = 128 (N).

  ADCSRA = 0x87;
  // turn on adc, freq = 1/128 , 125 kHz.  
  ADMUX = 0x60;
  //Bit 5 – ADLAR: ADC Left Adjust Result
  ADCSRA |= (1<<ADSC);
  //    while((ADCSRA&(1<<ADIF)) == 0); //Discard first conversion result.
  while(!(ADCSRA & 0x10));

  for(i=0; i<N; i++ ) {
    ADCSRA |= (1<<ADSC);
    //    while((ADCSRA&(1<<ADIF)) == 0);
    while(!(ADCSRA & 0x10));

    x[i] = ADCL;
    x[i] += (ADCH << 8);  
  }  

  ADCSRA = 0x00;

  for (i=0; i<N; i++) {
    x[i] -=  sdvig;
    if (i & 0x01) {
      fx[(N+i)>>1] = x[i] ;
    } else {
      fx[i>>1] = x[i] ;
    }
  }  

  //Performing FFT, getting fx[] array, where each element represents
  //frequency bin with width 65 Hz.

  fix_fftr( fx, log2N );

  // Calculation of the magnitude:
  for (i=0; i<N/2; i++) {
    fx[i] = sqrt((long)fx[i] * (long)fx[i] + (long)fx[i+N/2] * (long)fx[i+N/2]);
  }

  //Show data on three color LEDs display:

  kraccums = kraccumn;  //Save old data for each band RGB, for smoothing;
  zlaccums = zlaccumn;
  snaccums = snaccumn;

  kraccumn = 0;  //Reset 
  zlaccumn = 0;
  snaccumn = 0;

  for ( count = 1; count < 11; count++ ) {
    kraccumn = kraccumn + fx[count]; //Red band, no zero bin, don't use DC component.
  }

  for ( count = 11; count < 21; count++ ) {
    zlaccumn = zlaccumn + fx[count]; //Green band
  }

  for ( count = 21; count < 32; count++ ) {
    snaccumn = snaccumn + fx[count]; //Blue band
  }

  //Smoothing, so it fall down gradually.  
  if ( kraccumn < (kraccums * kdmp) ) {
    kraccumn = (kraccums * kdmp);
  }
  if ( zlaccumn < (zlaccums * kdmp) ) {
    zlaccumn = (zlaccums * kdmp);
  }
  if ( snaccumn < (snaccums * kdmp) ) {
    snaccumn = (snaccums * kdmp);
  }

  //Visual Display AGC for all three band
  if ( kraccumn > maxim || zlaccumn > maxim || snaccumn > maxim)  {
    vrem = max( kraccumn, zlaccumn );
    vrem = max( vrem, snaccumn );
    maxim = vrem ;
  } else {
    maxim *=   kary;
  }

  //Debugging monitor, allow to check processing data on each stage.
  // x command - printout data received from ADC (input raw data).
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 'x') {
      for (i=0; i<N/2; i++){
        Serial.print(x[i], DEC);
        Serial.print(", ");
        if ((i+1)%10 == 0) {
          Serial.print("\n");
        }
      } 
      Serial.print("\n");
      delay(200);
    }
  // f command - printout data after FFT. Clear view of each bin in the spectrum.
  // Plus printing summary accumulator for each band and variable MAXIM.
    if (incomingByte == 'f') {
      for (i=1; i<N/4; i++){
        Serial.print(fx[i], DEC);
        Serial.print(",");
        if ((i+1)%10 == 0) {
          Serial.print("\n");
        }
      } 
      Serial.print("\n Red accum. :");
      Serial.print(kraccumn, DEC);
      Serial.print("\n Green accum. :");
      Serial.print(zlaccumn, DEC);
      Serial.print("\n Blue accum. :");
      Serial.print(snaccumn, DEC);
      Serial.print("\n MAXIM: ");
      Serial.print(maxim, DEC);
      Serial.print("\n");
      Serial.println(" Array FFT printed.");
      delay(200);
    }

  //Adjustment Time constant of the AGC, depends on nature of the music.
    if (incomingByte == '1') {
      kary = 0.99;
      Serial.print("\n");
      Serial.println(" New value kary = 0.99.");
      delay(200);
    }
    if (incomingByte == '2') {
      kary = 0.999;
      Serial.print("\n");
      Serial.println(" New value kary = 0.999.");
      delay(200);
    }
    if (incomingByte == '3') {
      kary = 0.9999;
      Serial.print("\n");
      Serial.println(" New value kary = 0.9999.");
      delay(200);
    }
  // z command - stop any communication, handy command on Ubuntu.
    if (incomingByte == 'z') {
      Serial.println(" STOP in 2 sec.");
      delay(2000);
      Serial.end();
    }
  }
  // Display, 3 band (RGB), 4 level.
  int ledLevelKr = map(kraccumn, minim, maxim, 0, ledCountKr);
  int ledLevelZl = map(zlaccumn, minim, maxim, 0, ledCountZl);
  int ledLevelSn = map(snaccumn, minim, maxim, 0, ledCountSn);
  
  for (int thisLed = 0; thisLed < strip.numPixels(); thisLed++) {
    while (ledLevelKr > 0) {
      strip.setPixelColor(thisLed, 255, 0, 0);
      ledLevelKr--;
      thisLed++;
    }
    while (ledLevelZl > 0) {
      strip.setPixelColor(thisLed, 0, 255, 0);
      ledLevelZl--;
      thisLed++;
    }
    while (ledLevelSn > 0) {
      strip.setPixelColor(thisLed, 0, 0, 255);
      ledLevelSn--;
      thisLed++;
    }
    strip.setPixelColor(thisLed, 0, 0, 0);
  }
  
  strip.show();   
}
