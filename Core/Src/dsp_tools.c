#include "dsp_tools.h"

#include "arm_common_tables.h"

int fft(q15_t *output, q15_t *input, size_t N) {
    arm_rfft_instance_q15 fft_instance;
    if (arm_rfft_init_q15(&fft_instance, N, 0, 1) != ARM_MATH_SUCCESS) {
        return 0;
    }
    arm_rfft_q15(&fft_instance, input, output);
    return 1;
}

int ifft(q15_t *output, q15_t *input, size_t N) {
    arm_rfft_instance_q15 ifft_instance;
    if (arm_rfft_init_q15(&ifft_instance, N, 1, 1) != ARM_MATH_SUCCESS) {
        return 0;
    }
    arm_rfft_q15(&ifft_instance, input, output);
    return 1;
}
