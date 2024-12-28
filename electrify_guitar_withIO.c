#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define SAMPLE_RATE_HZ 8000.0f
// delay line / comb filter constants
#define DELAYLINE_MAXLENGTH 100000
#define COMB_ALPHA 0.6f
#define COMB_BETA 0.4f
#define COMB_DELAY_MS 500.0f
// IIR filter constants
#define IIR_const_1 0.5
// FIR low-pass filter (fc = fs / 4), 0.25 dB ripple in pass-band, 60dB attenuation in stop-band, ? taps
#define IFX_OVERDRIVE_LPF_INP_LENGTH 27
// pi lol
#define M_PI 3.14159265358979323846

volatile int *LEDR_ptr = (int *)0xFF200000;

// tremolo effect {
typedef struct {
  float depth;  // tremolo depth aka delta

  // low-freq oscillator (LFO) parameters
  float lfoDir;
  float lfoCount;
  float lfoCountLimit;

  // Effect sample rate (Hz)
  float sampleRate_Hz;

  // Effect output
  float out;

} Tremolo;

void Tremolo_Init(Tremolo *trem, float depth, float lfoFrequency_Hz,
                  float sampleRate_Hz) {
  // boundary check for tremolo depth
  if (depth < 0.0f) {
    depth = 0.0f;
  } else if (depth > 1.0f) {
    depth = 1.0f;
  }

  trem->depth = depth;  // set depth

  trem->sampleRate_Hz = sampleRate_Hz;  // set sample rate

  // boundary check for LFO frequency
  if (lfoFrequency_Hz < 0.0f) {
    lfoFrequency_Hz = 1.0f;
  } else if (lfoFrequency_Hz > 0.5f * trem->sampleRate_Hz) {
    lfoFrequency_Hz = 0.5f * trem->sampleRate_Hz;  // roof to Nyquist limit
  }

  trem->lfoCountLimit = 0.25f * trem->sampleRate_Hz / lfoFrequency_Hz;

  // ensure LFO counter is within new counter limit range
  if (trem->lfoCount > trem->lfoCountLimit) {
    trem->lfoCount = trem->lfoCountLimit;
  } else if (trem->lfoCount < -trem->lfoCountLimit) {
    trem->lfoCount = -trem->lfoCountLimit;
  }

  trem->lfoCount = 0;
  trem->lfoDir = 1;
  trem->out = 0.0f;
}

float Tremolo_Update(Tremolo *trem, float inp) {
  // modulate input signal
  trem->out =
      inp * ((1.0f - trem->depth) + trem->depth * trem->depth *
                                        (trem->lfoCount / trem->lfoCountLimit));

  // if we've reached a max of the triangle wave, count down
  if (trem->lfoCount >= trem->lfoCountLimit) {
    trem->lfoDir = -1.0f;
  } else if (trem->lfoCount <= -trem->lfoCountLimit) {
    trem->lfoDir = 1.0f;
  }

  // increment LFO counter
  trem->lfoCount += trem->lfoDir;

  return trem->out;
}
// } tremolo effect


// delay line effect {
typedef struct {
  // delay line length
  uint32_t length;

  // delay line circ buffer
  uint32_t index;
  float memory[DELAYLINE_MAXLENGTH];

} DelayLine;

void DelayLine_Init(DelayLine *dlyLn, float delayTime_ms, float sampleRate_Hz) {
  // compute delay line length
  dlyLn->length = (uint32_t)(0.001f * delayTime_ms * sampleRate_Hz);

  // clamp delay line length
  if (dlyLn->length > DELAYLINE_MAXLENGTH) dlyLn->length = DELAYLINE_MAXLENGTH;

  // clear delay line memory (buffer)
  for (uint32_t n = 0; n < dlyLn->length; n++) {
    dlyLn->memory[n] = 0.0f;
  }

  // clear delay line index
  dlyLn->index = 0;
}

float DelayLine_Update(DelayLine *dlyLn, float inp) {
  // read from delay line at current index
  float out = dlyLn->memory[dlyLn->index];

  // overwrite delay line at current index
  dlyLn->memory[dlyLn->index] = inp;

  // increment delay line index
  dlyLn->index++;

  // check for circ buff wrap
  if (dlyLn->index >= dlyLn->length) {
    dlyLn->index -= dlyLn->length;
  }

  // return delay line output
  return out;
}
// } delay line effect


// first-order IIR low pass filter {
typedef struct {
  float alpha;
  float out; // saves it for accessing prev output in eqn
} FirstOrderIIR;

void FirstOrderIIR_Init(FirstOrderIIR *filter, float alpha) {
  
  if (alpha < 0.0f) {
    filter->alpha = 0.0f;
  } else if (alpha > 1.0f) {
    filter->alpha = 1.0f;
  } else {
    filter->alpha = alpha;
  }

  filter->out = 0.0f; // reset output

}

float FirstOrderIIR_Update(FirstOrderIIR *filter, float in) {

  // compute output using current input and previous output
  filter->out = (1 - filter->alpha) * in + filter->alpha * filter->out;

  return filter->out;
}
// } first-order IIR low pass filter


// overdrive effect
static double filter_taps[IFX_OVERDRIVE_LPF_INP_LENGTH] = {
  -0.0050571904779315455,
  -0.01637631819471444,
  -0.013643921597646349,
  -0.0030300065782467994,
  0.017202939406826618,
  0.0276967455674842,
  0.013815885257775988,
  -0.023079206302511916,
  -0.055856915297031524,
  -0.04621048025160752,
  0.027379488388065365,
  0.14694801617169112,
  0.25875776276386037,
  0.30439758013783486,
  0.25875776276386037,
  0.14694801617169112,
  0.027379488388065365,
  -0.04621048025160752,
  -0.055856915297031524,
  -0.023079206302511916,
  0.013815885257775988,
  0.0276967455674842,
  0.017202939406826618,
  -0.0030300065782467994,
  -0.013643921597646349,
  -0.01637631819471444,
  -0.0050571904779315455
};

typedef struct {
  // sampling time
  float T;

  // input anti-aliasing low-pass filter (FIR)
  float lpfInpBuf[IFX_OVERDRIVE_LPF_INP_LENGTH]; // impulse response array
  uint8_t lpfInpBufIndex; // for managing convolution
  float lpfInpOut; // for passing result to next filter

  // input high-pass filter (IIR, first order)
  float hpfInpBufIn[2]; // [0] = new input x, [1] = prev input x
  float hpfInpBufOut[2]; // [0] = new input y, [1] = prev input y
  float hpfInpWcT; // filter constant
  float hpfInpOut; // for passing result to next filter

  // overdrive clipping
  float preGain;
  float threshold;

  float out; // final output
} IFX_Overdrive;

void IFX_Overdrive_Init(IFX_Overdrive *od, float samplingFrequencyHz, float hpfCutoffFrequencyHz, float odPreGain, float lpfCutoffFrequencyHz) {
  
  // sampling time
  od->T = 1.0f / samplingFrequencyHz;

  // input high-pass filter
  od->hpfInpBufIn[0] = 0.0f; od->hpfInpBufIn[1] = 0.0f;
  od->hpfInpBufOut[0] = 0.0f; od->hpfInpBufOut[1] = 0.0f;
  od->hpfInpWcT = 2.0f * M_PI * hpfCutoffFrequencyHz * od->T;
  od->hpfInpOut = 0.0f;

  // input low-pass filter
  for (uint8_t n = 0; n < IFX_OVERDRIVE_LPF_INP_LENGTH; n++) {
    od->lpfInpBuf[n] = 0;
  }
  od->lpfInpBufIndex = 0;
  od->lpfInpOut = 0.0f;

  // overdrive clipping
  od->preGain = odPreGain;
  od->threshold = 1.0f / 3.0f;
}

float IFX_Overdrive_Update(IFX_Overdrive *od, float inp) {
  
  // FIR low-pass anti-aliasing filter
  od->lpfInpBuf[od->lpfInpBufIndex] = inp;
  od->lpfInpBufIndex++;

  if (od->lpfInpBufIndex == IFX_OVERDRIVE_LPF_INP_LENGTH) {
    od->lpfInpBufIndex = 0;
  }

  od->lpfInpOut = 0.0f;
  uint8_t index = od->lpfInpBufIndex;
  for (uint8_t n = 0; n < IFX_OVERDRIVE_LPF_INP_LENGTH; n++) {
    if (index == 0) {
      index = IFX_OVERDRIVE_LPF_INP_LENGTH - 1;
    } else {
      index--;
    }

    od->lpfInpOut = filter_taps[n] * od->lpfInpBuf[index];
  }

  // first order IIR high-pass filter to remove low frequency components, which muddy when overdriven
  od->hpfInpBufIn[1] = od->hpfInpBufIn[0];
  od->hpfInpBufIn[0] = od->lpfInpOut;

  od->hpfInpBufOut[1] = od->hpfInpBufOut[0];
  od->hpfInpBufOut[0] = (2.0f * (od->hpfInpBufIn[0] - od->hpfInpBufIn[1]) + (2.0f - od->hpfInpWcT) * od->hpfInpBufOut[1]) / (2.0f + od->hpfInpWcT);
  od->hpfInpOut = od->hpfInpBufOut[0];

  // overdrive
  float clipIn = od->preGain * od->hpfInpOut;
  float absClipIn = fabs(clipIn);
  float signClipIn = (clipIn >= 0.0f) ? 1.0f : -1.0f;

  float clipOut = 0.0f;

  // test: magnitude scaling for DE1-SoC, which has +-20 million readings 
  float magnitudeScale = 1.2f * 10000000.0f; // 1.2 * 10 million
  od->threshold *= magnitudeScale;

  if (absClipIn < od->threshold) {
    clipOut = 2.0f * clipIn;
    // *(LEDR_ptr) = 0x001;
  } else if (absClipIn >= od->threshold && absClipIn < (2.0f * od->threshold)) {
    clipOut = signClipIn * (3.0f - (2.0f - 3.0f * absClipIn) * (2.0f - 3.0f * absClipIn)) / 3.0f;
    // *(LEDR_ptr) = 0x010;
  } else {
    clipOut = signClipIn * magnitudeScale;
    // *(LEDR_ptr) = 0x100;
  }

  od->out = clipOut;

  return od->out;
}


int main() {
  unsigned int fifospace;
  volatile int *audio_ptr = (int *)0xFF203040;  // audio port
  volatile int *switch_ptr = (int *)0xFF200040; // switch port
  printf("hi\n");

  Tremolo trem;
  Tremolo_Init(&trem, 0.5f, 5.0f, SAMPLE_RATE_HZ);
  // taking overdrive calculation into account
  // Tremolo_Init(&trem, 0.5f, (5.0f / 5.0f), SAMPLE_RATE_HZ);

  DelayLine dlyLn;
  DelayLine_Init(&dlyLn, COMB_DELAY_MS, SAMPLE_RATE_HZ);
  // taking overdrive calculation into account
  // DelayLine_Init(&dlyLn, (COMB_DELAY_MS / 5.0f), SAMPLE_RATE_HZ);

  IFX_Overdrive od;
                      //  sample rate hz, hpf cutoff, od preGain, lpf cutoff
  IFX_Overdrive_Init(&od, SAMPLE_RATE_HZ, 7000.0f,   50.0f,      2500.0f);

  *(LEDR_ptr) = 0x000;

  int useOverdrive = 0;
  int useDelay = 0;
  int useTremolo = 0;

  int switch_state = *switch_ptr;
  int switch_check_counter = 0;

  while (1) {
      // Check switches less frequently
      if (++switch_check_counter >= 10000) {
          switch_state = *switch_ptr;
          switch_check_counter = 0;
      }

      useOverdrive = ((switch_state & 0x001) == 0x001);
      useDelay = ((switch_state & 0x002) == 0x002);
      useTremolo = ((switch_state & 0x004) == 0x004);

      // Process audio sample
      fifospace = *(audio_ptr + 1);
      if ((fifospace & 0x000000FF) > 0 &&
          (fifospace & 0x00FF0000) > 0 &&
          (fifospace & 0xFF000000) > 0) {
          
          int sample = *(audio_ptr + 3);

          if (useOverdrive) {
              sample = IFX_Overdrive_Update(&od, sample);
          }
          if (useDelay) {
              sample = COMB_ALPHA * sample + COMB_BETA * DelayLine_Update(&dlyLn, sample);
          }
          if (useTremolo) {
              sample = Tremolo_Update(&trem, sample);
          }

          if (useOverdrive) {
            sample = 3 * sample; 
          } else {
            sample = sample / 6;
          }
          *(audio_ptr + 2) = sample;
          *(audio_ptr + 3) = sample;
      }
  }
}