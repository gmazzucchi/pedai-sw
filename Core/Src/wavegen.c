#include "wavegen.h"
#include "organ_presets.h"

const static double FREQUENCY_TABLE[100] = {
    16.35, 17.32222, 18.35225, 19.44354, 20.59971, 21.82463, 23.12239, 24.49732, 25.95401, 27.49731, 29.13239, 30.86469, 32.7, 34.64444, 36.70451, 38.88707, 41.19942, 43.64926, 46.24478, 48.99464, 51.90801, 54.99463, 58.26478, 61.72938, 65.4, 69.28889, 73.40902, 77.77415, 82.39884, 87.29853, 92.48957, 97.98928, 103.81603, 109.98925, 116.52955, 123.45876, 130.8, 138.57777, 146.81804, 155.54829, 164.79767, 174.59705, 184.97913, 195.97857, 207.63206, 219.9785, 233.0591, 246.91752, 261.6, 277.15555, 293.63607, 311.09658, 329.59535, 349.19411, 369.95827, 391.95713, 415.26412, 439.957, 466.11821, 493.83504, 523.2, 554.31109, 587.27214, 622.19316, 659.19069, 698.38821, 739.91654, 783.91426, 830.52823, 879.91401, 932.23642, 987.67008, 1046.4, 1108.62218, 1174.54429, 1244.38633, 1318.38139, 1396.77642, 1479.83307, 1567.82853, 1661.05646, 1759.82802, 1864.47284, 1975.34016, 2092.8, 2217.24436, 2349.08857, 2488.77265, 2636.76277, 2793.55285, 2959.66614, 3135.65705, 3322.11292, 3519.65604, 3728.94567, 3950.68032, 4185.6, 4434.48873, 4698.17715, 4977.5453
};


#if SOUND_PLAYER_I2S == SOUND_PLAYER_SAMPLED

#if PED_PHASE_VOCODER == PED_ENABLED

#define STFT_HOPLEN (32U)
#define STFT_SEGLEN (64U)
#define STFT_N_SEGS ((2U * MAX_NOTE_LEN / STFT_SEGLEN) - 1U)

#warning Not the cleanest way to do it
#define SAMPLE_LEN (SAMPLE_D2_22KHZ_CORPO_L)

static int16_t stft_bufs[STFT_N_SEGS][STFT_SEGLEN];

size_t phase_vocoder(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, size_t offset, int dsem) {
    // divide the signal to fill the stft_bufs
    // maybe we have to apply a window function...
    for (int isample = 0; isample < target_len; isample++) {
        int sample_idx          = (isample + offset) % SAMPLE_LEN;
        int blk_n               = 2 * isample / STFT_SEGLEN;
        int blk_i               = isample % STFT_SEGLEN;
        stft_bufs[blk_n][blk_i] = base_note[sample_idx];
        if (isample - STFT_HOPLEN > 0) {
            stft_bufs[blk_n + 1][(blk_i + STFT_HOPLEN) % STFT_SEGLEN] = base_note[sample_idx];
        }
    }

    const arm_cfft_instance_q15 cfft_instance = {
        .fftLen       = STFT_SEGLEN, /**< length of the FFT. */
        .pTwiddle     = NULL,        // const q15_t *pTwiddle;             /**< points to the Twiddle factor table. */
        .pBitRevTable = NULL,        // const uint16_t *pBitRevTable;      /**< points to the bit reversal table. */
        .bitRevLength = 0            /**< bit reversal table length. */
    };

    const arm_rfft_instance_q15 fft_instance = {
        .fftLenReal        = STFT_SEGLEN, /**< length of the real FFT. */
        .ifftFlagR         = 0,           /**< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
        .bitReverseFlagR   = 0, /**< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
        .twidCoefRModifier = 0, /**< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
        .pTwiddleAReal     = NULL,            // const q15_t *pTwiddleAReal;   /**< points to the real twiddle factor table. */
        .pTwiddleBReal     = NULL,            // const q15_t *pTwiddleBReal;   /**< points to the imag twiddle factor table. */
        .pCfft             = &cfft_instance,  // const arm_cfft_instance_q15 *pCfft;    /**< points to the complex FFT instance. */
    };

    for (size_t ibufs = 0; ibufs < STFT_N_SEGS; ibufs++) {
        arm_rfft_q15(&fft_instance, stft_bufs[ibufs], stft_bufs[ibufs]);
    }

    // now it's time to stretch and to linearly interpolate
    // arm_linear_interp_q15()

    const arm_cfft_instance_q15 inverse_cfft_instance = {
        .fftLen       = STFT_SEGLEN, /**< length of the FFT. */
        .pTwiddle     = NULL,        // const q15_t *pTwiddle;             /**< points to the Twiddle factor table. */
        .pBitRevTable = NULL,        // const uint16_t *pBitRevTable;      /**< points to the bit reversal table. */
        .bitRevLength = 0            /**< bit reversal table length. */
    };

    const arm_rfft_instance_q15 inverse_fft_instance = {
        .fftLenReal        = STFT_SEGLEN, /**< length of the real FFT. */
        .ifftFlagR         = 0,           /**< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
        .bitReverseFlagR   = 1, /**< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
        .twidCoefRModifier = 0, /**< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
        .pTwiddleAReal     = NULL,                    // const q15_t *pTwiddleAReal;   /**< points to the real twiddle factor table. */
        .pTwiddleBReal     = NULL,                    // const q15_t *pTwiddleBReal;   /**< points to the imag twiddle factor table. */
        .pCfft             = &inverse_cfft_instance,  // const arm_cfft_instance_q15 *pCfft;    /**< points to the complex FFT instance. */
    };

    // arm_rfft_q15(&inverse_fft_instance, fft_buf, target_note);
    for (size_t ibufs = 0; ibufs < STFT_N_SEGS; ibufs++) {
        arm_rfft_q15(&inverse_fft_instance, stft_bufs[ibufs], stft_bufs[ibufs]);
    }

    // restore
}

size_t attacco_pitch_shifting(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, size_t offset, int dsem) {
    size_t len = phase_vocoder(target_note, target_len, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, offset, dsem);
#warning TODO: implement the attacco directly from the corpo
    return target_len;
}

size_t corpo_pitch_shifting(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, size_t offset, int dsem) {
    return phase_vocoder(target_note, target_len, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, offset, dsem);
}

size_t decay_pitch_shifting(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, size_t offset, int dsem) {
    // take base note and pitch shift
    size_t len = phase_vocoder(target_note, target_len, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, offset, dsem);
    // then apply decay effect
    double TAU = (((double)target_len) / 15.0);
    for (size_t idx = 0; idx < target_len; idx++) {
        target_note[idx] = target_note[idx] * exp(-(idx / TAU));
    }
    return target_len;
}

#else   // PED_PHASE_VOCODER != PED_ENABLED
/* // WORKING PYTHON ALGORITHM
 lp = LOOP_POINT_DATA[sample]["lp_start"]
ls = LOOP_POINT_DATA[sample]["lp_stop"]
(frames, nframes) = soundfile.read(sample + ".wav")
attacco_frames = frames[:lp]
corpo_frames = frames[lp:ls]
dsem = 4
ratio = 2**(dsem/12)
IL = len(corpo_frames)
L = math.trunc(len(corpo_frames) / ratio)
corpo_frames_2 = [0] * L
for i in range(0, L):
    x = (i * ratio)
    y = x - math.trunc(x)
    z = math.trunc(x) % IL
    corpo_frames_2[i] = corpo_frames[z] * \
        (1 - y) + corpo_frames[(z + 1) % IL] * y */

int attacco_pitch_shifting(int16_t *curr, size_t curr_len, int16_t *base, size_t base_len, int dsem) {
    if (dsem == 0) {
        return 0;
    }
    double ratio = pow(2.0, dsem / 12.0);
    size_t L     = base_len / ratio;
    for (size_t i = 0; i < L; i++) {
        double x = i * ratio;
        size_t y = (size_t)x;
        size_t z = (size_t)x % base_len;
        curr[i]  = base[z] * (1 - y) + base[(z + 1) % base_len] * y;
    }
    return 0;
}

// returns the new current note length
size_t corpo_pitch_shifting(int16_t *curr, size_t curr_max_len, int16_t *base, size_t base_len, int dsem) {
    if (dsem == 0) {
        if (curr_max_len < base_len) {
            return 0;
        }
        memcpy(curr, base, base_len * sizeof(int16_t));
        return base_len;
    }
    long double ratio = powl(2.0, dsem / 12.0);
    size_t L          = (size_t)((double)base_len / ratio);
    if (curr_max_len < L) {
        // TODO: better error handling
        L = curr_max_len;
        // Error_Handler();
    }
    // TODO: try different interpolation functions from the arm_math.h library
    // arm_linear_interp_instance_f32 config;
    // arm_linear_interp_f32(&config, 0.0f);
    // arm_linear_interp_instance_f32 instance = {
    //     .nValues = L,
    //     .pYData = base, // technically double = float64
    //     .x1 = 1,
    //     .xSpacing = 1
    // };
    for (size_t i = 0; i < L; i++) {
        double x = (double)i * ratio;
        double y = x - (size_t)x;
        size_t z = (size_t)x % base_len;
        curr[i]  = base[z] * (1 - y) + base[(z + 1) % base_len] * y;
    }
    // TODO: implement also time_stretching
    return L;
}
#endif  // PED_PHASE_VOCODER == PED_ENABLED

/**
 * @brief Given the set of keys, it composes the waveform data to be played via I2S
 * 
 * @param nstate_keys The bitset for the current set of keys pressed
 * @param pstate_keys The bitset for the previous set of keys pressed
 * @param current_note Frame of the note
 * @param current_note_max_len Maximum length for the note frame
 * @return size_t 
 */
size_t compose_note(const uint32_t nstate_keys, const uint32_t pstate_keys, int16_t *current_note, size_t current_note_max_len) {
    // placeholder for testing
    // memcpy(current_note, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L * sizeof(uint16_t));
    // return SAMPLE_D2_22KHZ_CORPO_L;

    /***
     * for example:
     * pstate_keys 0110111 // keys before
     * nstate_keys 1010011 // keys now
     * 
     * keep_n 0010011 // keys held down
     * new_ns 1000100 // keys just pressed
     * old_ns 0100100 // keys just released
     */
    const uint32_t keep_notes = nstate_keys & pstate_keys;  // do the corpo
    const uint32_t new_notes  = nstate_keys ^ keep_notes;   // do the attacco
    const uint32_t old_notes  = pstate_keys ^ keep_notes;   // do the rilascio
    int16_t tmp_adder[MAX_NOTE_LEN];
    int16_t tmp_time_stretcher[MAX_NOTE_LEN];
    size_t tmp_adder_len          = 0;
    size_t tmp_time_stretcher_len = 0;
    char display_notes_buf[BUFSIZ];
    size_t display_ptr = 0;

    memset(current_note, 0, current_note_max_len);

    size_t ctst    = MAX_NOTE_LEN;  // maybe: min(MAX_NOTE_LEN, current_note_max_len);  // chosen_time_stretching_target
    size_t n_notes = 0;

#if PED_PHASE_VOCODER == PED_ENABLED

    for (int inote = 0; inote < n_bitnotes; inote++) {
        int bit_to_check = n_bitnotes - inote;
        if ((new_notes >> bit_to_check) & 1) {
            size_t to_add =
                attacco_pitch_shifting(tmp_adder, ctst, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, note_buffer_position[inote], inote);
            // TODO: check ctst == return value of pitch shifting, but we can assume it
            note_buffer_position[inote] = (note_buffer_position[inote] + to_add) % SAMPLE_LEN;
            arm_add_q15(current_note, tmp_adder, current_note, to_add);
        } else if ((keep_notes >> bit_to_check) & 1) {
            size_t to_add =
                corpo_pitch_shifting(tmp_adder, ctst, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, note_buffer_position[inote], inote);
            // TODO: check ctst == return value of pitch shifting, but we can assume it
            note_buffer_position[inote] = (note_buffer_position[inote] + to_add) % SAMPLE_LEN;
            arm_add_q15(current_note, tmp_adder, current_note, to_add);
        } else if ((old_notes >> bit_to_check) & 1) {
            size_t to_add =
                decay_pitch_shifting(tmp_adder, ctst, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, note_buffer_position[inote], inote);
            // TODO: check ctst == return value of pitch shifting, but we can assume it
            note_buffer_position[inote] = (note_buffer_position[inote] + to_add) % SAMPLE_LEN;
            arm_add_q15(current_note, tmp_adder, current_note, to_add);
        } else {
            // note is not played so reset the note_buffer_position[inote]
            note_buffer_position[inote] = 0;
        }

        if ((nstate_keys >> bit_to_check) & 1) {
            int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%s ", note_names[inote]);
            display_ptr += to_add;
        }
    }

    lcd_1602a_write_text(display_notes_buf);

#if SCALE_AMPLITUDE_AFTER_ADDING == PED_ENABLED
    if (n_notes > 1) {
        arm_scale_q15(current_note, 0xFFFF / n_notes, 0, current_note, current_note_max_len);
    }
#endif

    return ctst;

#else

    for (bitnotes_t inote = bitnote_c1; inote < n_bitnotes; inote++) {
        int bit_to_check = n_bitnotes - inote;
        /* 
            As it should be...
            if ((nstate_keys >> isem) & 1) {
                int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%s ", note_names[note_d + isem]);
                display_ptr += to_add;
            }

            if ((new_notes >> isem) & 1) {
                tmp_adder_len        = corpo_pitch_shifting(tmp_adder, CURRENT_NOTE_L, sample_D2_22kHz_attacco, SAMPLE_D2_22KHZ_ATTACCO_L, isem);
                current_note_max_len = min(current_note_max_len, tmp_adder_len);
                arm_add_q15(current_note, tmp_adder, current_note, current_note_max_len);
                n_notes++;
            } else if ((keep_notes >> isem) & 1) {
                tmp_adder_len         = corpo_pitch_shifting(tmp_adder, CURRENT_NOTE_L, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, isem);
                current_note_max_len = min(current_note_max_len, tmp_adder_len);
                arm_add_q15(current_note, tmp_adder, current_note, current_note_max_len);
                n_notes++;
            } 
        */

        if ((new_notes >> bit_to_check) & 1) {
            // TODO: attacco
            tmp_adder_len        = corpo_pitch_shifting(tmp_adder, MAX_NOTE_LEN, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, inote);
            current_note_max_len = min(current_note_max_len, tmp_adder_len);
            arm_add_q15(current_note, tmp_adder, current_note, current_note_max_len);
            n_notes++;
            int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%s ", note_names[inote]);
            display_ptr += to_add;
        } else if ((keep_notes >> bit_to_check) & 1) {
            // the note is the same
        }
    }

    // int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%lu %lu %lu P %u N %u", keep_notes, new_notes, old_notes, pstate_keys, nstate_keys);
    // display_ptr += to_add;
    lcd_1602a_write_text(display_notes_buf);

#if SCALE_AMPLITUDE_AFTER_ADDING == PED_ENABLED
    // consider whether to include it
    if (n_notes > 1) {
        arm_scale_q15(current_note, 0xFFFF / n_notes, 0, current_note, current_note_max_len);
    }
#endif

    return current_note_max_len;
#endif
}

#elif SOUND_PLAYER_I2S == SOUND_PLAYER_SYNTH

/**
 * @brief Given the set of keys, it composes the waveform data to be played via I2S
 * 
 * @param nstate_keys The bitset for the current set of keys pressed
 * @param pstate_keys The bitset for the previous set of keys pressed
 * @param current_note Frame of the note
 * @param current_note_max_len Maximum length for the note frame
 * @return size_t 
 */
size_t compose_note(bool *nstate_keys, bool *pstate_keys, int16_t *current_note, size_t current_note_max_len) {
#define AMPLITUDE 128
    memset(current_note, 0, current_note_max_len * sizeof(int16_t));

    static size_t is_offset[N_HW_KEYS] = {0};

    bool keep_notes[N_HW_KEYS];  // = nstate_keys & pstate_keys;
    for (size_t inote = 0; inote < N_HW_KEYS; inote++) {
        keep_notes[inote] = nstate_keys[inote] & pstate_keys[inote];
    }

    bool new_notes[N_HW_KEYS];  // nstate_keys ^ keep_notes;
    for (size_t inote = 0; inote < N_HW_KEYS; inote++) {
        new_notes[inote] = nstate_keys[inote] ^ pstate_keys[inote];
    }

    bool old_notes[N_HW_KEYS];  // pstate_keys ^ keep_notes;
    for (size_t inote = 0; inote < N_HW_KEYS; inote++) {
        old_notes[inote] = pstate_keys[inote] ^ keep_notes[inote];
    }

    for (size_t inote = 0; inote < N_HW_KEYS; inote++) {
        if (keep_notes[inote] || new_notes[inote]) {
            const double cfreq      = BASE_FREQUENCY * powl(2, (inote / 12.0));
            const size_t cis_offset = 0;  // is_offset[inote]; // doesn't work for the above optimization layer
            for (size_t ris = 0; ris < current_note_max_len; ris++) {
                const size_t is            = ris + cis_offset;
                const double sample_period = AUDIO_FREQUENCY_HZ / cfreq;
                tmp_adder[ris]             = arm_sin_q15(2 * PI * is / sample_period) * AMPLITUDE;
            }
            is_offset[inote] += current_note_max_len;
            arm_add_q15(tmp_adder, current_note, current_note, current_note_max_len);
            memset(tmp_adder, 0, current_note_max_len * sizeof(int16_t));
        }
        if (old_notes[inote]) {
            is_offset[inote] = 0;
        }
    }
    return current_note_max_len;
}

#elif SOUND_PLAYER_I2S == SOUND_PLAYER_ADDSYNTH

arm_status compute_ifft(q15_t *current_note, const size_t note_fft_len) {
    arm_cfft_instance_q15 fft_instance;
    arm_status status = arm_cfft_init_q15(&fft_instance, note_fft_len);
    
    if (status != ARM_MATH_SUCCESS) return status;
    arm_cfft_q15(&fft_instance, current_note, 1, 1);
    /***
     * It feels so wrong to skip normalization after ifft computation...
     */
    // arm_scale_q15(current_note, 1, log2(note_fft_len), current_note, 2*note_fft_len);
    for (size_t isample = 0; isample < note_fft_len; isample++) {
        current_note[isample] = current_note[2*isample];
    }    
    return ARM_MATH_SUCCESS;
}

void add_frequency_components_for_a_note(q15_t* spec, const size_t note_fft_len, const double* organ_preset, 
                                         const size_t organ_preset_size, const size_t kidx) {
    double total_gain = 0;
    for (size_t h = 0; h < organ_preset_size; h++) {
        total_gain += organ_preset[h];
    }
    for (size_t h = 0; h < organ_preset_size; h++) {
        double f0 = FREQUENCY_TABLE[kidx] * (h + 1);
        const size_t N = note_fft_len;
        size_t bin = (size_t) round(f0*h*N/AUDIO_FREQUENCY_HZ);
        if (bin >= N/2) break;
        /***
         * I feel removing scaling to total_gain is wrong, 
         * because sum(organ_preset) > 1 and should be normalized in theory
         */
        // q15_t A = (q15_t) (organ_preset[h] * (32768.0) / total_gain);
        q15_t A = (q15_t) (organ_preset[h] * (32768.0));

        spec[2*bin + 0] += A;     // Real
        spec[2*bin + 1] += 0;     // Imaginary
        spec[2*(N-bin) + 0] += A; // Symmetric real
        spec[2*(N-bin) + 1] += 0; // Symmetric imaginary
    }
}
/**
 * TODO: to fix, this is broken
size_t sample_sinusoid(bool *nstate_keys, bool *nstate_pedals, q15_t* current_note, size_t current_note_len) {
    float f0 = 221;
    float fs = AUDIO_FREQUENCY_HZ;
    float period = fs / f0;
    for (size_t is = 0; is < current_note_len; is++) {
        // TODO: this is the wrong part surely
        q15_t x = (q15_t)((is / (2.0f * PI)) * 32768.0f);   
        current_note[is] = arm_sin_q15(x);
    }
    return current_note_len;
}
*/

size_t compose_note(bool *pstate_keys, bool *nstate_keys, bool *pstate_pedals, bool *nstate_pedals, q15_t* current_note, size_t max_current_note_len) {
#if 0
/**
 * To test the sound player mechanism, first compose a sample sinusoid in function of the key states...
 */
    return sample_sinusoid(nstate_keys, nstate_pedals, current_note, current_note_len);
#endif
    const size_t note_fft_len = 4096U;
    
    if (max_current_note_len < note_fft_len * 2) {
        return 0;
    }
    memset(current_note, 0, note_fft_len * 2 * sizeof(current_note[0]));
    
    for (size_t kidx = 0; kidx < N_HW_KEYS; kidx++) {
        if (!nstate_keys[kidx]) continue;
        add_frequency_components_for_a_note(current_note, note_fft_len, ORGAN_PRESET_3, ORGAN_PRESET_3_SIZE, kidx);
    }

    for (size_t pidx = 0; pidx < N_HW_PEDAL_KEYS; pidx++) {
        if (!nstate_pedals[pidx]) continue;
        add_frequency_components_for_a_note(current_note, note_fft_len, ORGAN_PRESET_3, ORGAN_PRESET_3_SIZE, pidx);
    }
    compute_ifft(current_note, note_fft_len);
    
    q15_t max_value = 0;
    uint32_t max_idx;
    arm_max_q15(current_note, note_fft_len, &max_value, &max_idx);
#define MAX_IDEAL_VALUE_A_RECIA (10000U)
    int8_t n_scaling = MAX_IDEAL_VALUE_A_RECIA > max_value 
                        ? (MAX_IDEAL_VALUE_A_RECIA / max_value) 
                        : (0 - (max_value / MAX_IDEAL_VALUE_A_RECIA));
#if 0
    /***
     * A futura memoria, 
     * arm_scale_q15 deve moltiplicare due q15_t, che sono in pratica due int16_t.
     * Per fare ciò li mette temporaneamente in un q31_t, li moltiplica insieme e poi tiene 
     * i 15 bit piu significativi.
     * Se moltiplico per 5 e basta, sicuro mi servono anche i bit non significativi,
     * altrimenti il risultato viene troncato. Per fare ciò devo shiftare a sinistra
     * di 15 dopo la moltiplicazione e allora diventano bit significativi e il risultato
     * viene preservato.
     * Bisogna ragionare con gli int16_t e non con il float scalato da -1 a 1.
     * Lascio qua il codice di debug come reperto.
     */
#include <stdio.h>
    printf("The max value was %d at index %d\r\n", (int)max_value, (int)max_idx);
    q15_t scale_factor = (1 << 15) / max_value;
    arm_scale_q15(current_note, 1 << 15, 5, current_note, current_note_len);
    for (size_t i = 0; i < note_fft_len; i++) {
        current_note[i] = current_note[i] << 5;
    }
    arm_shift_q15(current_note, 1, current_note, note_fft_len);
    arm_scale_q15(current_note, n_scaling, 15, current_note, note_fft_len);
    arm_max_q15(current_note, note_fft_len, &max_value, &max_idx);
    printf("The max value is now %d at index %d\r\n", (int)max_value, (int)max_idx);
#endif

    arm_scale_q15(current_note, n_scaling, 15, current_note, note_fft_len);
    return note_fft_len;
}

#endif
