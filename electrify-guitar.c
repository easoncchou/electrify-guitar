#include <stdio.h>

#define SAMPLE_RATE_HZ 48000.0f

// tremolo effect {
typedef struct {
    
    float depth; // tremolo depth aka delta
    
    // low-freq oscillator (LFO) parameters
    float lfoDir;
    float lfoCount;
    float lfoCountLimit;

    // Effect sample rate (Hz)
    float sampleRate_Hz;

    // Effect output
    float out;

} Tremolo;

void Tremolo_Init(Tremolo *trem, float depth, float lfoFrequency_Hz, float sampleRate_Hz) {
    
    // boundary check for tremolo depth
    if (depth < 0.0f) {
        depth = 0.0f;
    } else if (depth > 1.0f) {
        depth = 1.0f;
    }

    trem->depth = depth; // set depth

    trem->sampleRate_Hz = sampleRate_Hz; // set sample rate

    // boundary check for LFO frequency
    if (lfoFrequency_Hz < 0.0f) {
        lfoFrequency_Hz = 1.0f;
    } else if (lfoFrequency_Hz > 0.5f * trem->sampleRate_Hz) {
        lfoFrequency_Hz = 0.5f * trem->sampleRate_Hz; // roof to Nyquist limit
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
    trem->out = inp * ( (1.0f - trem->depth) + trem->depth * trem->depth * (trem->lfoCount / trem->lfoCountLimit) );

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

int main()
{
    unsigned int fifospace;
    volatile int * audio_ptr = (int *) 0xFF203040; // audio port
    printf("hi\n");
    Tremolo trem;
    Tremolo_Init(&trem, 0.5f, 20.0f, SAMPLE_RATE_HZ);

    while (1)
    {
        fifospace = *(audio_ptr+1); // read the audio port fifospace register
        if ((fifospace & 0x000000FF) > 0 &&        // Available sample right
            (fifospace & 0x00FF0000) > 0 &&        // Available write space right
            (fifospace & 0xFF000000) > 0)        // Available write space left
        {
            int sample = *(audio_ptr + 3);    // read right channel only
            // printf("sample: %d\n", sample);
            sample = Tremolo_Update(&trem, sample);
            sample = sample / 2;
            *(audio_ptr + 2) = sample;        // Write to both channels
            *(audio_ptr + 3) = sample;
        }
    }
}