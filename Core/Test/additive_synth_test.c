#include <stdio.h>
#include "arm_math.h"

int main(void)
{
    q15_t x = 8192;  // pi/2
    q15_t y = arm_sin_q15(x);

    printf("sin(pi/2) Q15 = %d\n", y);
    printf("sin(pi/2) float = %.6f\n", y / 32768.0f);

    return 0;
}
