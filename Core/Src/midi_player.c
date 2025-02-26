#include "midi_player.h"

#if PED_USB_DEVICE_CLASS == PED_USB_MIDI_CLASS

#include "tusb.h"

#define MIDI_NOTE_ON  (0x90)
#define MIDI_NOTE_OFF (0x80)

void midi_player_init() {
    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
}

void midi_player_update(bool *pstate, bool *nstate) {
    /* 
        // This means only one HAL_Delay(286) at the program start
        static uint32_t start_ms = 0;
        if (get_current_time_ms() - start_ms < 286) {
            return;
        }
        start_ms += 286; 
    */

    /* 
        The MIDI interface always creates input and output port/jack descriptors
        regardless of these being used or not. Therefore incoming traffic should be read
        (possibly just discarded) to avoid the sender blocking in IO 
    */
    while (tud_midi_available()) {
        uint8_t packet[4];
        tud_midi_packet_read(packet);
    }

    /***
        This is an example of the logic:
        pstate 0110111 // keys before
        nstate 1010011 // keys now

        keep_n 0010011 // keys held down
        new_ns 1000100 // keys just pressed
        old_ns 0100100 // keys just released
     */
    // const uint32_t keep_notes = nstate & pstate;      // do the corpo
    // const uint32_t new_notes  = nstate ^ keep_notes;  // do the attacco
    // const uint32_t old_notes  = pstate ^ keep_notes;  // do the rilascio

    /***
     * Same logic but using the arrays
     */
    bool keep_notes[N_HW_KEYS];
    for (size_t inote = 0; inote < N_HW_KEYS; inote++) {
        keep_notes[inote] = nstate[inote] && pstate[inote];
    }
    bool new_notes[N_HW_KEYS];
    for (size_t inote = 0; inote < N_HW_KEYS; inote++) {
        new_notes[inote] = (nstate[inote] ^ keep_notes[inote]) & 1;
    }
    bool old_notes[N_HW_KEYS];
    for (size_t inote = 0; inote < N_HW_KEYS; inote++) {
        old_notes[inote] = (pstate[inote] ^ keep_notes[inote]) & 1;
    }

    uint8_t const cable_num = 0;  // MIDI jack associated with USB endpoint
    uint8_t const channel   = 0;  // 0 for channel 1

    // release all the old notes
    for (size_t is = 1; is <= n_bitnotes; is++) {
        bool note_is_released = old_notes[is];  // with bitset: (old_notes >> (n_bitnotes - is) & 1);
        if (note_is_released) {
            uint8_t note_off[3] = {MIDI_NOTE_OFF | channel, is + base_midinote, 0};
            tud_midi_stream_write(cable_num, note_off, 3);
        }
    }

    // press all the new notes
    for (size_t is = 1; is <= n_bitnotes; is++) {
        bool note_is_pressed = new_notes[is];  // with bitset: (new_notes >> (n_bitnotes - is) & 1);
        if (note_is_pressed) {
            uint8_t note_on[3] = {MIDI_NOTE_ON | channel, is + base_midinote, 127};
            tud_midi_stream_write(cable_num, note_on, 3);
        }
    }
}

#endif  // PED_USB_DEVICE_CLASS == PED_USB_MIDI_CLASS
