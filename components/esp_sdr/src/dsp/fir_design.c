#include "fir_design.h"

#include <math.h>

#define KAISER_BETA 5.65f
#define STAGE1_KERNEL_TAPS 18u
#define STAGE1_TRIANGLE_TAPS 15u

/* Modified Bessel function I0, power series. */
static float bessel_i0(float x)
{
    float sum = 1.0f, term = 1.0f;
    for (int k = 1; k < 30; k++) {
        float q = x / (2.0f * (float)k);
        term *= q * q;
        sum += term;
    }
    return sum;
}

static float kaiser_sinc_tap(uint32_t i, uint32_t count, float cutoff)
{
    float m = (float)i - 0.5f * (float)(count - 1);
    float x = 2.0f * cutoff * m;
    float sinc = fabsf(x) < 1e-6f ? 1.0f : sinf((float)M_PI * x) / ((float)M_PI * x);
    float r = 2.0f * m / (float)(count - 1);
    float window = bessel_i0(KAISER_BETA * sqrtf(fmaxf(0.0f, 1.0f - r * r))) / bessel_i0(KAISER_BETA);
    return 2.0f * cutoff * sinc * window;
}

static void quantize(int16_t *out, const float *taps, uint32_t count)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < count; i++) {
        sum += taps[i];
    }
    for (uint32_t i = 0; i < count; i++) {
        out[i] = (int16_t)lrintf(32768.0f * taps[i] / sum);
    }
}

void fir_design_stage1(int16_t taps[FIR_STAGE1_TAPS])
{
    float kernel[STAGE1_KERNEL_TAPS], h[FIR_STAGE1_TAPS] = {0};
    for (uint32_t i = 0; i < STAGE1_KERNEL_TAPS; i++) {
        kernel[i] = kaiser_sinc_tap(i, STAGE1_KERNEL_TAPS, 0.5f / (float)FIR_STAGE1_DECIMATION);
    }
    for (uint32_t t = 0; t < STAGE1_TRIANGLE_TAPS; t++) {
        float weight = (float)(t < 8u ? t + 1u : STAGE1_TRIANGLE_TAPS - t);
        for (uint32_t i = 0; i < STAGE1_KERNEL_TAPS; i++) {
            h[t + i] += weight * kernel[i];
        }
    }
    quantize(taps, h, FIR_STAGE1_TAPS);
}

void fir_design_lowpass(int16_t *taps, uint32_t count, uint32_t decimation)
{
    float h[FIR_STAGE2_MAX_TAPS];
    for (uint32_t i = 0; i < count; i++) {
        h[i] = kaiser_sinc_tap(i, count, 0.5f / (float)decimation);
    }
    quantize(taps, h, count);
}

uint32_t fir_stage2_taps(uint32_t d2)
{
    uint32_t taps = (18u * d2 + 7u) & ~7u;
    return taps > FIR_STAGE2_MAX_TAPS ? FIR_STAGE2_MAX_TAPS : taps;
}
