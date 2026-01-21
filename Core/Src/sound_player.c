#include "sound_player.h"

#include "arm_math.h"
#include "lcd1602a.h"
#include "main.h"
#include "samples/sample_22kHz_D2.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static bool ready_to_play_note                   = true;
static bool has_to_play_note                     = false;
static bool has_to_change_note                   = false;
static volatile bool sai_transfer_completed      = false;
static volatile bool sai_is_transmitting         = false;
static volatile bool sai_half_transfer_completed = false;

static int16_t tmp_adder[MAX_NOTE_LEN]         = {0};
static int16_t buffer0_sai[MAX_NOTE_LEN]       = {0};
static int16_t buffer1_sai[MAX_NOTE_LEN]       = {0};
static int16_t *sound_data_db[2]               = {buffer0_sai, buffer1_sai};
static size_t sound_data_db_len[2]             = {0, 0};
static bool active_b                           = 0;
static size_t note_buffer_position[n_bitnotes] = {0};

const double FREQUENCY_TABLE[100] = {
    16.35, 17.32222, 18.35225, 19.44354, 20.59971, 21.82463, 23.12239, 24.49732, 25.95401, 27.49731, 29.13239, 30.86469, 32.7, 34.64444, 36.70451, 38.88707, 41.19942, 43.64926, 46.24478, 48.99464, 51.90801, 54.99463, 58.26478, 61.72938, 65.4, 69.28889, 73.40902, 77.77415, 82.39884, 87.29853, 92.48957, 97.98928, 103.81603, 109.98925, 116.52955, 123.45876, 130.8, 138.57777, 146.81804, 155.54829, 164.79767, 174.59705, 184.97913, 195.97857, 207.63206, 219.9785, 233.0591, 246.91752, 261.6, 277.15555, 293.63607, 311.09658, 329.59535, 349.19411, 369.95827, 391.95713, 415.26412, 439.957, 466.11821, 493.83504, 523.2, 554.31109, 587.27214, 622.19316, 659.19069, 698.38821, 739.91654, 783.91426, 830.52823, 879.91401, 932.23642, 987.67008, 1046.4, 1108.62218, 1174.54429, 1244.38633, 1318.38139, 1396.77642, 1479.83307, 1567.82853, 1661.05646, 1759.82802, 1864.47284, 1975.34016, 2092.8, 2217.24436, 2349.08857, 2488.77265, 2636.76277, 2793.55285, 2959.66614, 3135.65705, 3322.11292, 3519.65604, 3728.94567, 3950.68032, 4185.6, 4434.48873, 4698.17715, 4977.5453
};

inline size_t min(size_t x, size_t y) {
    return (x > y) ? (y) : (x);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    sai_transfer_completed = true;
    sai_is_transmitting    = false;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    sai_half_transfer_completed = true;
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
    uint32_t error_code = HAL_I2S_GetError(hi2s);
    char log_buf[BUFSIZ];
    snprintf(log_buf, BUFSIZ, "%lu", error_code);
    lcd_1602a_write_text(log_buf);
}

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

// lontanamente simile a una doppia ancia
const double ORGAN_PRESET_1[] = {0.1, 0.03, 0.05, 0.04, 0.05, 0.01, 0.015, 0.02, 0.02, 0.01, 0.02};
// un principale dolce    
const double ORGAN_PRESET_2[] = {0.2, 0.1, 0, 0, 0.05, 0.1, 0.03, 0.02, 0.01};
// registro dolce
// amps2 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1]
// un misto tra un principale e un ripieno
const double ORGAN_PRESET_3[] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

arm_status compute_ifft(q15_t *current_note, const size_t note_fft_len, const size_t note_fft_sample_len) {
    arm_cfft_instance_q15 fft_instance;
    arm_status status = arm_cfft_init_q15(&fft_instance, note_fft_len);
    if (status != ARM_MATH_SUCCESS) return status;
    arm_cfft_q15(&fft_instance, current_note, 1, 1);
    return ARM_MATH_SUCCESS;
}

void add_frequency_components_for_a_note(q15_t* spec, const size_t note_fft_len, const double* organ_preset, const size_t kidx) {
    for (size_t preset_idx = 0; preset_idx < (sizeof(organ_preset)/sizeof(organ_preset[0])); organ_preset++) {
        double f = FREQUENCY_TABLE[kidx] * (preset_idx + 1);
        int bin_idx = (int)(f * note_fft_len / 22000);
        double A = ORGAN_PRESET_1[preset_idx] / 2.0;
        // TODO: This is surely WRONG!!!
        spec[bin_idx] += A;
        spec[-bin_idx] += A;
    }
}

size_t compose_note(bool *pstate_keys, bool *nstate_keys, bool *pstate_pedals, bool *nstate_pedals, int16_t* current_note, size_t current_note_len) {

    const size_t note_fft_len = 4096U;
    const size_t note_fft_sample_len = note_fft_len * 2;
    
    if (current_note_len < note_fft_sample_len) {
        // TODO: ERROR
        return 0;
    }
    memset(current_note, 0, current_note_len * sizeof(note_fft_sample_len));
    
    for (size_t kidx = 0; kidx < N_HW_KEYS; kidx++) {
        if (!nstate_keys) continue;
        add_frequency_components_for_a_note(current_note, note_fft_len, ORGAN_PRESET_3, kidx);
    }

    for (size_t pidx = 0; pidx < N_HW_PEDAL_KEYS; pidx++) {
        if (!nstate_pedals) continue;
        add_frequency_components_for_a_note(current_note, note_fft_len, ORGAN_PRESET_3, pidx);
    }
    compute_ifft(current_note, note_fft_len, note_fft_sample_len);
    

    // TODO: normalization
#if 0
    q15_t maxValue = 0;
    size_t maxIdx;
    arm_max_q15(current_note, current_note_len, &maxValue, &maxIdx);
    arm_scale_q15(current_note, maxValue, 0, current_note, current_note_len);
#endif
}

#endif

#if SOUND_PLAYER_I2S != PED_DISABLED

void sound_player_init(void) {
    memset(tmp_adder, 0, sizeof(tmp_adder));
    memset(buffer0_sai, 0, sizeof(buffer0_sai));
    memset(buffer1_sai, 0, sizeof(buffer1_sai));
}

void sound_player_routine(bool *pstate_keys, bool *nstate_keys, bool *pstate_pedals, bool *nstate_pedals) {
    bool zero_buffer[N_HW_KEYS] = {0};

    if (!has_to_play_note && sai_is_transmitting) {
        // stop the sound
        HAL_I2S_DMAStop(&hi2s1);  // HAL_SAI_DMAStop(&hsai_BlockA1);
        sai_half_transfer_completed = false;
        sai_transfer_completed      = false;
        sai_is_transmitting         = false;

        ready_to_play_note = true;
        HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET);
    } else if (has_to_play_note && sai_transfer_completed) {
        sai_transfer_completed = false;
        sai_is_transmitting    = true;

        HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)sound_data_db[active_b], (uint16_t)sound_data_db_len[active_b]);
        HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

    } else if (has_to_play_note && ready_to_play_note) {
        ready_to_play_note  = false;
        sai_is_transmitting = true;

        HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)sound_data_db[active_b], (uint16_t)sound_data_db_len[active_b]);
        HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);
    } else if (has_to_change_note) {
        has_to_change_note  = false;
        ready_to_play_note  = false;
        sai_is_transmitting = true;

        HAL_I2S_DMAStop(&hi2s1);  // HAL_SAI_DMAStop(&hsai_BlockA1);
        HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)sound_data_db[active_b], (uint16_t)sound_data_db_len[active_b]);

        HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);
    }

#define ALL_KEYS_RELEASED (0)
    if (memcmp(pstate_keys, zero_buffer, N_HW_KEYS) == 0 && memcmp(nstate_keys, zero_buffer, N_HW_KEYS) == 0) {
        // np to np
        // ensure that no note is played
        lcd_1602a_clear_screen();
        has_to_play_note = false;
    }
    // else if (pstate_keys == 0b1111 && nstate_keys != 0b1111) { // Covered in the last case
    // }
    else if ((memcmp(pstate_keys, zero_buffer, N_HW_KEYS) != 0 && memcmp(nstate_keys, zero_buffer, N_HW_KEYS) == 0) &&
             (memcmp(pstate_pedals, zero_buffer, N_HW_PEDAL_KEYS) != 0 && memcmp(nstate_pedals, zero_buffer, N_HW_PEDAL_KEYS) == 0)) {
        // p to np
        // stop at the next iteration
        has_to_play_note = false;
    } else if ((memcmp(nstate_keys, pstate_keys, N_HW_KEYS) == 0) && memcmp(nstate_pedals, pstate_pedals, N_HW_PEDAL_KEYS)) {
        // p same note
        // continue with the same buffer
        has_to_play_note = true;
        // memcpy(doublebuffer_sai[!active_buffer_sai], doublebuffer_sai[active_buffer_sai], doublebuffer_sai_len[active_buffer_sai] * sizeof(int16_t));
    } else {
        // np to p
        // p different note
        // construct the note in the inactive buffer and then swap the buffer at the next iteration
        // has_to_play_note                         = true;
        has_to_change_note           = true;
        sound_data_db_len[!active_b] = compose_note(pstate_keys, nstate_keys, pstate_pedals, nstate_pedals, sound_data_db[!active_b], MAX_NOTE_LEN);
        active_b                     = !active_b;
    }
}

#endif
