#include <stdint.h>
#include <stdio.h>

#define SAMPLE_RATE_HZ 8000.0f
#define DELAYLINE_MAXLENGTH 100000
#define COMB_ALPHA 0.7f
#define COMB_BETA 0.3f
#define COMB_DELAY_MS 500.0f

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

int main() {
  unsigned int fifospace;
  volatile int *audio_ptr = (int *)0xFF203040;  // audio port
  printf("hi\n");

  Tremolo trem;
  Tremolo_Init(&trem, 0.5f, 5.0f, SAMPLE_RATE_HZ);

  DelayLine dlyLn;
  DelayLine_Init(&dlyLn, COMB_DELAY_MS, SAMPLE_RATE_HZ);

  while (1) {
    fifospace = *(audio_ptr + 1);  // read the audio port fifospace register
    if ((fifospace & 0x000000FF) > 0 &&  // Available sample right
        (fifospace & 0x00FF0000) > 0 &&  // Available write space right
        (fifospace & 0xFF000000) > 0)    // Available write space left
    {
      int sample = *(audio_ptr + 3);  // read right channel only

      // printf("sample: %d\n", sample);

      // comb filter using delay line
      sample = COMB_ALPHA * sample + COMB_BETA * DelayLine_Update(&dlyLn, sample);
    
      sample = Tremolo_Update(&trem, sample);

      sample = sample / 3;
      *(audio_ptr + 2) = sample;  // Write to both channels
      *(audio_ptr + 3) = sample;
    }
  }
}