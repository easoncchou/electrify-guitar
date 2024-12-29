# Electrify - Guitar Amplifier / Effects Pedal

Takes mic input (for example from an acoustic guitar) to apply DSP audio algorithms and emulate effects seen in electric guitar amplifiers and their pedals.

Made for the DE1-SoC running the Intel FPGA Monitor Program.

## Effects

## Tremolo (Amplitude Modulation)

A tremolo effect that modulates the volume between loud and quiet.

Settings available for tremolo depth (how much variance there is between loud and quiet) and tremolo frequency (how fast the volume oscillates between loud and quiet).

## Delay Line (Singular Echo)

A delay line that replays a singular echo of a previous input.

Settings available for the time lag between current and previous sound, and parameters for the balance between volume of current and past input.

## Overdrive (Filtering and Soft-clipping Distortion)

The overdrive effect employs the following effects in this order:

**FIR High-Pass Filter** to improve overall sound quality

**IIR Low-Pass Filter** to mitigate lower frequency sounds that get muddied when clipped

**Soft-Clipping via Digitization of Analog Parallel-Diode Model** to distort the audio and give it a fuzzy quality

### Running / Implementations

There is a version that activates the effects by interfacing the switches, but memory access times seem to mess with the sound and the parameters were not tuned for this difference, so the overdrive effect sounds worse.
