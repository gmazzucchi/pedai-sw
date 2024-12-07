#include "key_reader.h"

#include "main.h"

#define N_MUX_CONTROL_PINS (4U)
#define N_MUX_DATA_PINS    (4U)

static GPIO_TypeDef *mux_c_ports[N_MUX_CONTROL_PINS] = {
    MUX_C3_GPIO_Port,
    MUX_C2_GPIO_Port,
    MUX_C1_GPIO_Port,
    MUX_C0_GPIO_Port,
};

const static uint16_t mux_c_pins[N_MUX_CONTROL_PINS] = {
    MUX_C3_Pin,
    MUX_C2_Pin,
    MUX_C1_Pin,
    MUX_C0_Pin,
};

/* static GPIO_TypeDef *mux_d_ports[N_MUX_DATA_PINS] = {
    MUX_D0_GPIO_Port,
    MUX_D1_GPIO_Port,
    MUX_D2_GPIO_Port,
    MUX_D3_GPIO_Port,
};

const static uint16_t mux_d_pins[N_MUX_DATA_PINS] = {
    MUX_D0_Pin,
    MUX_D1_Pin,
    MUX_D2_Pin,
    MUX_D3_Pin,
}; */

void set_mux_addr(int addr) {
    for (int i = 0; i < N_MUX_CONTROL_PINS; i++) {
        HAL_GPIO_WritePin(mux_c_ports[i], mux_c_pins[i], (addr >> i) & 1);
    }
}

uint32_t read_keys() {
    uint32_t nstate = 0;
    set_mux_addr(0);
    return HAL_GPIO_ReadPin(MUX_D0_GPIO_Port, MUX_D0_Pin);
    /* for (int iaddr = 0; iaddr < (1 << N_MUX_CONTROL_PINS); iaddr++) {
        set_mux_addr(iaddr);
        if (!HAL_GPIO_ReadPin(MUX_D0_GPIO_Port, MUX_D0_Pin)) {
            nstate |= 1 << iaddr;
        }
    } */
    return nstate;
}
