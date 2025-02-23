#ifndef DSP_TOOLS_H
#define DSP_TOOLS_H

#include "ped_config.h"
#include "arm_math.h"

int fft(q15_t* output, q15_t* input, size_t N);
int ifft(q15_t* output, q15_t* input, size_t N);


#endif // DSP_TOOLS_H

