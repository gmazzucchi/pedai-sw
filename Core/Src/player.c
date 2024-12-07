#include "player.h"
#include "lcd1602a.h"
#include "samples/sample_22kHz_D2.h"

#include "arm_math.h"
#include "main.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static bool ready_to_play_note                   = true;
static bool has_to_play_note                     = false;
static bool has_to_change_note                   = false;
static volatile bool sai_transfer_completed      = false;
static volatile bool sai_is_transmitting         = false;
static volatile bool sai_half_transfer_completed = false;

static int16_t tmp_adder[MAX_NOTE_LEN];
static int16_t buffer0_sai[MAX_NOTE_LEN];
static int16_t buffer1_sai[MAX_NOTE_LEN];
static int16_t *sound_data_db[2]   = {buffer0_sai, buffer1_sai};
static size_t sound_data_db_len[2] = {0, 0};
static bool active_b               = 0;

static const char note_names[bitnotes_n_ped_notes][4] = {
    "c1", "c1#", "d1", "e1b", "e1", "f1", "f1#", "g1", "g1#", "a1", "b1b", "b1",
    "c2", "c2#", "d2", "e2b", "e2", "f2", "f2#", "g2", "g2#", "a2", "b2b", "b2",
};

inline size_t min(size_t x, size_t y) {
    return (x > y) ? (y) : (x);
}

#if defined(PED_PHASE_VOCODER_ENABLED)

size_t phase_vocoder(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, int dsem) {
    // // Apply Hann window
    // hann_window(hann, FRAME_SIZE);
    // for (int i = 0; i < FRAME_SIZE; i++) {
    //     signal[i] *= hann[i];
    // }

    // // Perform FFT
    // fft(signal, FRAME_SIZE);

    int16_t fft_buf[CURRENT_NOTE_L];

    const arm_cfft_instance_q15 cfft_instance = {
        .fftLen       = target_len, /**< length of the FFT. */
        .pTwiddle     = NULL,       // const q15_t *pTwiddle;             /**< points to the Twiddle factor table. */
        .pBitRevTable = NULL,       // const uint16_t *pBitRevTable;      /**< points to the bit reversal table. */
        .bitRevLength = 0           /**< bit reversal table length. */
    };

    const arm_rfft_instance_q15 fft_instance = {
        .fftLenReal        = target_len, /**< length of the real FFT. */
        .ifftFlagR         = 0,          /**< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
        .bitReverseFlagR   = 0, /**< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
        .twidCoefRModifier = 0, /**< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
        .pTwiddleAReal     = NULL,            // const q15_t *pTwiddleAReal;   /**< points to the real twiddle factor table. */
        .pTwiddleBReal     = NULL,            // const q15_t *pTwiddleBReal;   /**< points to the imag twiddle factor table. */
        .pCfft             = &cfft_instance,  // const arm_cfft_instance_q15 *pCfft;    /**< points to the complex FFT instance. */
    };

    arm_rfft_q15(&fft_instance, base_note, fft_buf);

    // Time-stretch: Modify phase and overlap
    // [Details omitted for brevity—phase unwrapping, modification]

    const arm_cfft_instance_q15 inverse_cfft_instance = {
        .fftLen       = target_len, /**< length of the FFT. */
        .pTwiddle     = NULL,       // const q15_t *pTwiddle;             /**< points to the Twiddle factor table. */
        .pBitRevTable = NULL,       // const uint16_t *pBitRevTable;      /**< points to the bit reversal table. */
        .bitRevLength = 0           /**< bit reversal table length. */
    };

    const arm_rfft_instance_q15 inverse_fft_instance = {
        .fftLenReal        = target_len, /**< length of the real FFT. */
        .ifftFlagR         = 0,          /**< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
        .bitReverseFlagR   = 1, /**< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
        .twidCoefRModifier = 0, /**< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
        .pTwiddleAReal     = NULL,            // const q15_t *pTwiddleAReal;   /**< points to the real twiddle factor table. */
        .pTwiddleBReal     = NULL,            // const q15_t *pTwiddleBReal;   /**< points to the imag twiddle factor table. */
        .pCfft             = &cfft_instance,  // const arm_cfft_instance_q15 *pCfft;    /**< points to the complex FFT instance. */
    };

    arm_rfft_q15(&inverse_fft_instance, fft_buf, target_note);

    // // Overlap and Add (OLA)
    // // [Details omitted for brevity]
}

size_t attacco_pitch_shifting(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, int dsem) {
    return phase_vocoder(target_note, target_len, sample_D2_22kHz_attacco, SAMPLE_D2_22KHZ_ATTACCO_L, dsem);
}

size_t corpo_pitch_shifting(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, int dsem) {
    return phase_vocoder(target_note, target_len, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, dsem);
}

size_t decay_pitch_shifting(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, int dsem) {
    // take base note and pitch shift
    size_t len = phase_vocoder(target_note, target_len, sample_D2_22kHz_attacco, SAMPLE_D2_22KHZ_ATTACCO_L, dsem);
    // then apply decay effect
    double TAU = (((double)target_len) / 15.0);
    for (size_t idx = 0; idx < target_len; idx++) {
        target_note[idx] = target_note[idx] * exp(-(idx / TAU));
    }
    return target_len;
}

#else
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
        Error_Handler();
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
#endif  // PED_PHASE_VOCODER_ENABLED

/**
 * @brief Given the set of keys, it composes the waveform data to be played via I2S
 * 
 * @param nstate The bitset for the current set of keys pressed
 * @param pstate The bitset for the previous set of keys pressed
 * @param current_note Frame of the note
 * @param current_note_max_len Maximum length for the note frame
 * @return size_t 
 */
size_t compose_note(unsigned int nstate, unsigned int pstate, int16_t *current_note, size_t current_note_max_len) {
    // placeholder

    // memcpy(current_note, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L * sizeof(uint16_t));
    // return SAMPLE_D2_22KHZ_CORPO_L;
    // return apply_window_crossfading(current_note, SAMPLE_D2_22KHZ_CORPO_L); // doesn't sound good...

    /***
     * pstate 0110111
     * nstate 1010011
     * 
     * keep_n 0010011
     * new_ns 1000100
     * old_ns 0100100
     */
    uint32_t keep_notes = nstate & pstate;      // do the corpo
    uint32_t new_notes  = nstate ^ keep_notes;  // do the attacco
    uint32_t old_notes  = pstate ^ keep_notes;  // do the rilascio
    int16_t tmp_adder[MAX_NOTE_LEN];
    int16_t tmp_time_stretcher[MAX_NOTE_LEN];
    size_t tmp_adder_len          = 0;
    size_t tmp_time_stretcher_len = 0;
    char display_notes_buf[BUFSIZ];
    size_t display_ptr = 0;

    memset(current_note, 0, current_note_max_len);

    size_t ctst    = SAMPLE_D2_22KHZ_CORPO_L;  // chosen_time_stretching_target
    size_t n_notes = 0;

#if defined(PED_PHASE_VOCODER_ENABLED)

    for (size_t isem = 0; isem < 12; isem++) {
        if ((new_notes >> isem) & 1) {
            attacco_pitch_shifting(tmp_adder, ctst, sample_D2_22kHz_attacco, SAMPLE_D2_22KHZ_ATTACCO_L, isem);
            // TODO: check ctst == return value of pitch shifting
            arm_add_q15(current_note, tmp_adder, current_note, ctst);
        } else if ((keep_notes >> isem) & 1) {
            corpo_pitch_shifting(tmp_adder, ctst, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, isem);
            // TODO: check ctst == return value of pitch shifting
            arm_add_q15(current_note, tmp_adder, current_note, ctst);
        }
        /* 
            else if ((old_notes >> isem) & 1) {
                decay_pitch_shifting(tmp_adder, ctst, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, isem);
                // TODO: check ctst == return value of pitch shifting
                arm_add_q15(current_note, tmp_adder, current_note, ctst);
        } 
        */

        if ((new_notes >> isem) & 1 && (keep_notes >> isem) & 1) {
            int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%s ", note_names[note_d + isem]);
            display_ptr += to_add;
        }
    }

    lcd_1602a_write_text(display_notes_buf);

    if (n_notes > 1)
        arm_scale_q15(current_note, 0xFFFF / n_notes, 0, current_note, current_note_max_len);

    return ctst;

#else

    for (size_t isem = 0; isem < 12; isem++) {
        /* 
            As it should be...
            if ((nstate >> isem) & 1) {
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

        if ((new_notes >> isem) & 1) {
            tmp_adder_len        = corpo_pitch_shifting(tmp_adder, MAX_NOTE_LEN, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, isem);
            current_note_max_len = min(current_note_max_len, tmp_adder_len);
            arm_add_q15(current_note, tmp_adder, current_note, current_note_max_len);
            n_notes++;
            int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%s ", note_names[bitnote_d1 + isem]);
            display_ptr += to_add;
        } else if ((keep_notes >> isem) & 1) {
        }
    }

    // int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%lu %lu %lu P %u N %u", keep_notes, new_notes, old_notes, pstate, nstate);
    // display_ptr += to_add;
    lcd_1602a_write_text(display_notes_buf);

    // consider whether to include it
    // if (n_notes > 1)
    // arm_scale_q15(current_note, 0xFFFF / n_notes, 0, current_note, current_note_max_len);

    return current_note_max_len;
#endif
}

void player_init() {
    memset(tmp_adder, 0, MAX_NOTE_LEN * sizeof(int16_t));
    memset(buffer0_sai, 0, MAX_NOTE_LEN * sizeof(int16_t));
    memset(buffer1_sai, 0, MAX_NOTE_LEN * sizeof(int16_t));
}

void player_routine(uint32_t pstate, uint32_t nstate) {
    if (!has_to_play_note && sai_is_transmitting) {
        // stop the sound
        HAL_I2S_DMAStop(&hi2s1);  // HAL_SAI_DMAStop(&hsai_BlockA1);
        sai_half_transfer_completed = false;
        sai_transfer_completed      = false;
        sai_is_transmitting         = false;

        ready_to_play_note = true;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    } else if (has_to_play_note && sai_transfer_completed) {
        sai_transfer_completed = false;
        sai_is_transmitting    = true;

        HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)sound_data_db[active_b], (uint16_t)sound_data_db_len[active_b]);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

    } else if (has_to_play_note && ready_to_play_note) {
        ready_to_play_note  = false;
        sai_is_transmitting = true;

        HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)sound_data_db[active_b], (uint16_t)sound_data_db_len[active_b]);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    } else if (has_to_change_note) {
        has_to_change_note  = false;
        ready_to_play_note  = false;
        sai_is_transmitting = true;

        HAL_I2S_DMAStop(&hi2s1);  // HAL_SAI_DMAStop(&hsai_BlockA1);
        HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)sound_data_db[active_b], (uint16_t)sound_data_db_len[active_b]);

        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }

#define ALL_KEYS_RELEASED (0)
    if (pstate == ALL_KEYS_RELEASED && nstate == ALL_KEYS_RELEASED) {
        // np to np
        // ensure that no note is played
        lcd_1602a_clear_screen();
        has_to_play_note = false;
    }
    // else if (pstate == 0b1111 && nstate != 0b1111) { // Covered in the last case
    // }
    else if (pstate != ALL_KEYS_RELEASED && nstate == ALL_KEYS_RELEASED) {
        // p to np
        // stop at the next iteration
        has_to_play_note = false;
    } else if (pstate == nstate) {
        // p same note
        // continue with the same buffer
        has_to_play_note = true;
        // memcpy(doublebuffer_sai[!active_buffer_sai], doublebuffer_sai[active_buffer_sai], doublebuffer_sai_len[active_buffer_sai] * sizeof(int16_t));
    } else {
// TODO: avoid interrupting the wave, either wait for the DMA to finish transmitting or start the note with an offset (very cool ngl)
#warning avoid interrupting the wave
        // np to p
        // p different note
        // construct the note in the inactive buffer and then swap the buffer at the next iteration
        // has_to_play_note                         = true;
        has_to_change_note           = true;
        sound_data_db_len[!active_b] = compose_note(nstate, pstate, sound_data_db[!active_b], MAX_NOTE_LEN);
        active_b                     = !active_b;
    }
    pstate = nstate;
}
