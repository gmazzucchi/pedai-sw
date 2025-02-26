#ifndef DSP_TOOLS_H
#define DSP_TOOLS_H

#include "arm_math.h"
#include "ped_config.h"

int fft(q15_t *output, q15_t *input, size_t N);
int ifft(q15_t *output, q15_t *input, size_t N);

#endif  // DSP_TOOLS_H
