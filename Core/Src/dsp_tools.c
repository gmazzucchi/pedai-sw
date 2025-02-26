#include "dsp_tools.h"

#include "arm_common_tables.h"

int fft(q15_t *output, q15_t *input, size_t N) {
    // q15_t twiddleCoef[N];
    // for (size_t i = 0; i< 3*N/4; i++) {
    //     twiddleCoef[2*i]   = cos(i * 2*PI/(float)N);
    //     twiddleCoef[2*i+1] = sin(i * 2*PI/(float)N);
    // }
    // const arm_cfft_instance_q15 cfft_instance = {
    //     .fftLen       = N,           /**< length of the FFT. */
    //     .pTwiddle     = twiddleCoef, // const q15_t *pTwiddle;             /**< points to the Twiddle factor table. */
    //     .pBitRevTable = armBitRevTable,  // const uint16_t *pBitRevTable;      /**< points to the bit reversal table. */
    //     .bitRevLength = 0            /**< bit reversal table length. */
    // };

    // const arm_rfft_instance_q15 fft_instance = {
    //     .fftLenReal        = N, /**< length of the real FFT. */
    //     .ifftFlagR         = 0,           /**< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
    //     .bitReverseFlagR   = 0, /**< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
    //     .twidCoefRModifier = 0, /**< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
    //     .pTwiddleAReal     = NULL,            // const q15_t *pTwiddleAReal;   /**< points to the real twiddle factor table. */
    //     .pTwiddleBReal     = NULL,            // const q15_t *pTwiddleBReal;   /**< points to the imag twiddle factor table. */
    //     .pCfft             = &cfft_instance,  // const arm_cfft_instance_q15 *pCfft;    /**< points to the complex FFT instance. */
    // };
    arm_rfft_instance_q15 fft_instance;
    if (arm_rfft_init_q15(&fft_instance, N, 0, 1) != ARM_MATH_SUCCESS) {
        return 0;
    }
    arm_rfft_q15(&fft_instance, input, output);
    return 1;
}

int ifft(q15_t *output, q15_t *input, size_t N) {
}
