#include "key_reader.h"

#include "main.h"

#define N_MUX_CONTROL_PINS (4U)
#define N_MUX_DATA_PINS    (4U)

static GPIO_TypeDef *mux_c_ports[N_MUX_CONTROL_PINS] = {
    MUX_C2_GPIO_Port,
    MUX_C1_GPIO_Port,
    MUX_C0_GPIO_Port,
};

const static uint16_t mux_c_pins[N_MUX_CONTROL_PINS] = {
    MUX_C2_Pin,
    MUX_C1_Pin,
    MUX_C0_Pin,
};

static GPIO_TypeDef *mux_d_ports[N_MUX_DATA_PINS] = {
    MUX1_DATA_GPIO_Port,
    MUX2_DATA_GPIO_Port,
    MUX3_DATA_GPIO_Port,
    MUX4_DATA_GPIO_Port,
    MUX5_DATA_GPIO_Port,
    MUX6_DATA_GPIO_Port,
};

const static uint16_t mux_d_pins[N_MUX_DATA_PINS] = {
    MUX1_DATA_Pin,
    MUX2_DATA_Pin,
    MUX3_DATA_Pin,
    MUX4_DATA_Pin,
    MUX5_DATA_Pin,
    MUX6_DATA_Pin,
};

static GPIO_TypeDef *cols_ports[8] = {
    C0_GPIO_Port,
    C0_GPIO_Port,
    C0_GPIO_Port,
    C0_GPIO_Port,
    C0_GPIO_Port,
    C0_GPIO_Port,
    C0_GPIO_Port,
    C0_GPIO_Port,
};

const static uint16_t cols_pins[8] = {
    C0_Pin,
    C0_Pin,
    C0_Pin,
    C0_Pin,
    C0_Pin,
    C0_Pin,
    C0_Pin,
    C0_Pin,
};


void set_mux_addr(int addr) {
    for (int i = 0; i < N_MUX_CONTROL_PINS; i++) {
        HAL_GPIO_WritePin(mux_c_ports[i], mux_c_pins[i], (addr >> i) & 1);
    }
}

int get_mat1_pin(int p) {
    if (p < 0 || p > 7) { 
        return -1;
    }
    if (p == 0) {
        return HAL_GPIO_ReadPin(R0_GPIO_Port, R0_Pin);
    } 
    if (p == 1) {
        return HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin);
    } 
    set_mux_addr(p-2);
    return HAL_GPIO_ReadPin(MUX5_DATA_GPIO_Port, MUX5_DATA_Pin);
}

void set_mat1_pin(int p, int s) {
    if (p < 0 || p > 7) return;
    if (p == 0) {
        HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, s);
    } else if (p == 1) {
        HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, s);
    } else {
        set_mux_addr(p-2);
        HAL_GPIO_WritePin(MUX5_DATA_GPIO_Port, MUX5_DATA_Pin, s);
    }
}

int get_mat2_pin(int p) {
    if (p < 0 || p > 7) return -1;
    return HAL_GPIO_ReadPin(cols_ports[p], cols_pins[p]);
}

void set_mat2_pin(int p, int s) {
    if (p < 0 || p > 7) return;
    HAL_GPIO_WritePin(cols_ports[p], cols_pins[p], GPIO_PIN_RESET);
}

// #define SWAP_ROWS_AND_COLS

void init_keys() {
/*     for (int c = 0; c < N_HW_MAT_COLS; c++) {
#ifdef SWAP_ROWS_AND_COLS
        set_mat1_pin(c, GPIO_PIN_RESET);
#else
        set_mat2_pin(c, GPIO_PIN_RESET);
#endif
    } */
   for (int c = 0; c < N_HW_MAT_COLS; c++) {
#ifdef SWAP_ROWS_AND_COLS
        set_mat1_pin(c, GPIO_PIN_SET);
#else
        set_mat2_pin(c, GPIO_PIN_SET);
#endif
    }
    set_mat1_pin(3, GPIO_PIN_SET);
    while(1);
}

void read_keys(bool* S) {
    /***
     * WITH DIODES MATRIX
    */
    for (int c = 0; c < N_HW_MAT_COLS; c++) {
#ifdef SWAP_ROWS_AND_COLS
        set_mat1_pin(c, GPIO_PIN_RESET);
#else
        set_mat2_pin(c, GPIO_PIN_RESET);    
#endif
        for (int r = 0; r < N_HW_MAT_ROWS; r++) {
#ifdef SWAP_ROWS_AND_COLS
            int v = get_mat2_pin(r);
#else
            int v = get_mat1_pin(r);
#endif
            S[c * 8 + r] = v;
        }
#ifdef SWAP_ROWS_AND_COLS
        set_mat1_pin(c, GPIO_PIN_SET);
#else
        set_mat2_pin(c, GPIO_PIN_SET);    
#endif
    } 
    // */

    /* 
        for (int c = 0; c < 8; c++) {
            if (c == 0) {
                HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, GPIO_PIN_RESET);
            } else if (c == 1) {
                HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);
            } else {
                set_mux_addr(c-2);
                HAL_GPIO_WritePin(MUX5_DATA_GPIO_Port, MUX5_DATA_Pin, GPIO_PIN_RESET);
            }
            for (int r = 0; r < 8; r++) {
                int v = 0;
                v = HAL_GPIO_ReadPin(cols_ports[r], cols_pins[r]);

                if (!v) {
                    nstate |= 1 << (c * 8 + r);
                }
            }
             if (c == 0) {
                HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, GPIO_PIN_SET);
            } else if (c == 1) {
                HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
            } else {
                set_mux_addr(c-2);
                HAL_GPIO_WritePin(MUX5_DATA_GPIO_Port, MUX5_DATA_Pin, GPIO_PIN_SET);
            }
        } 
    */

    /***
     * WITH MULTIPLEXERS
    */

    // set_mux_addr(0);
    // return HAL_GPIO_ReadPin(MUX_D0_GPIO_Port, MUX_D0_Pin);
    /* for (int iaddr = 0; iaddr < (1 << N_MUX_CONTROL_PINS); iaddr++) {
        set_mux_addr(iaddr);
        if (!HAL_GPIO_ReadPin(MUX_D0_GPIO_Port, MUX_D0_Pin)) {
            nstate |= 1 << iaddr;
        }
    } */
}
