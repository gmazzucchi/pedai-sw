import os
import json
import soundfile
import math

AMPLITUDE_CUTOFF = 2**5
LOOP_POINT_DATA = json.load(open("loop_points.json", "r"))


def normalize(data: list) -> list:
    return [int(elem * 32767 / AMPLITUDE_CUTOFF) for elem in data]


def generate_c_code(fp: str, frames: list, loop_start: int, loop_stop: int):
    filename = "sample_22kHz_" + fp
    header = open(os.path.join("sample_headers", filename + ".h"), "w+")
    source = open(os.path.join("sample_sources", filename + ".c"), "w+")
    FP = fp.upper()
    attacco_frames = ','.join(str(x) for x in frames[:loop_start])
    corpo_frames = ','.join(str(x) for x in frames[loop_start:loop_stop])
    header.write(f'''#ifndef SAMPLE_{FP}_22KHZ_H
#define SAMPLE_{FP}_22KHZ_H

#include <stdint.h>

#define SAMPLE_{FP}_22KHZ_ATTACCO_L {str(loop_start)}
#define SAMPLE_{FP}_22KHZ_CORPO_L {str(loop_stop - loop_start)}

extern int16_t sample_{fp}_22kHz_attacco[SAMPLE_{FP}_22KHZ_ATTACCO_L];
extern int16_t sample_{fp}_22kHz_corpo[SAMPLE_{FP}_22KHZ_CORPO_L];

#endif // SAMPLE_{FP}_22KHZ_H

''')
    source.write(f'''#include "{filename}.h"

int16_t sample_{fp}_22kHz_attacco[SAMPLE_{FP}_22KHZ_ATTACCO_L] = {{
    {attacco_frames}
}};

int16_t sample_{fp}_22kHz_corpo[SAMPLE_{FP}_22KHZ_CORPO_L] = {{
    {corpo_frames}
}};
''')


def accordo(n1, n2):
    L = min(len(n1), len(n2))
    n3 = [0] * L
    for i in range(0, L-1):
        n3[i] = n1[i] + n2[i]
    n4 = [*n3, *n3, *n3, *n3]
    soundfile.write("test_1.wav", n4, samplerate=22050)


# Si gode
def augmentation_test(sample: str):
    lp = LOOP_POINT_DATA[sample]["lp_start"]
    ls = LOOP_POINT_DATA[sample]["lp_stop"]
    (frames, nframes) = soundfile.read(sample + ".wav")
    attacco_frames = frames[:lp]
    corpo_frames = frames[lp:ls]
    dsem = 4
    ratio = 2**(dsem/12)
    """ for (size_t iattacco = 0; iattacco < c_attacco_len; iattacco++) {
    double x = (iattacco * ratio);
    double y = x - (int)x;
    size_t z = (size_t)x % (c_attacco_len);
    current_note[iattacco] = current_note_old[z] * (1 - y) +
                             current_note_old[(z + 1) % c_attacco_len] * y;
  } """

    IL1 = len(attacco_frames)
    L1 = math.trunc(len(attacco_frames) / ratio)
    attacco_frames_2 = [0] * L1
    for i in range(0, L1):
        x = (i * ratio)
        y = x - math.trunc(x)
        z = math.trunc(x) % IL1
        attacco_frames_2[i] = attacco_frames[z] * \
            (1 - y) + attacco_frames[(z + 1) % IL1] * y

    IL = len(corpo_frames)
    L = math.trunc(len(corpo_frames) / ratio)
    corpo_frames_2 = [0] * L
    for i in range(0, L):
        x = (i * ratio)
        y = x - math.trunc(x)
        z = math.trunc(x) % IL
        corpo_frames_2[i] = corpo_frames[z] * \
            (1 - y) + corpo_frames[(z + 1) % IL] * y

    # accordo(corpo_frames, corpo_frames_2)
    final_frames = [*attacco_frames_2, *corpo_frames_2, *corpo_frames_2, *corpo_frames_2, *corpo_frames_2, *corpo_frames_2,
                    *corpo_frames_2, *corpo_frames_2, *corpo_frames_2, *corpo_frames_2, *corpo_frames_2, *corpo_frames_2, *corpo_frames_2]
    soundfile.write("test.wav", final_frames, samplerate=22050)


def generate_all_c_code():
    files = os.listdir()

    for file in files:
        if file.endswith(".wav"):
            [fp, _] = file.split(".")
            (x, y) = soundfile.read(file)
            lp = LOOP_POINT_DATA[fp]["lp_start"]
            ls = LOOP_POINT_DATA[fp]["lp_stop"]
            generate_c_code(fp.replace("#", "_sharp"), normalize(x), lp, ls)


if __name__ == "__main__":
    # generate_all_c_code()
    augmentation_test("A2")
