#include "key_reader.h"

// #include "adc.h"
#include "main.h"
#include "ped_config.h"

#define N_MUX_CONTROL_PINS (3U)
#define N_MUX_DATA_PINS    (4U)

static GPIO_TypeDef *mux_c_ports[N_MUX_CONTROL_PINS] = {
    MUX_C0_GPIO_Port,
    MUX_C1_GPIO_Port,
    MUX_C2_GPIO_Port,
};

const static uint16_t mux_c_pins[N_MUX_CONTROL_PINS] = {
    MUX_C0_Pin,
    MUX_C1_Pin,
    MUX_C2_Pin,
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

static GPIO_TypeDef *cols_ports[N_HW_MAT_COLS] = {
    C7_GPIO_Port,
    C6_GPIO_Port,
    C5_GPIO_Port,
    C4_GPIO_Port,
    C3_GPIO_Port,
    C2_GPIO_Port,
    C1_GPIO_Port,
    C0_GPIO_Port,
};

const static uint16_t cols_pins[N_HW_MAT_COLS] = {
    C7_Pin,
    C6_Pin,
    C5_Pin,
    C4_Pin,
    C3_Pin,
    C2_Pin,
    C1_Pin,
    C0_Pin,
};

void set_mux_addr(int addr) {
    for (int i = 0; i < N_MUX_CONTROL_PINS; i++) {
        HAL_GPIO_WritePin(mux_c_ports[i], mux_c_pins[i], (addr >> i) & 1);
    }
}

int get_matrix_row_pin(int p) {
    if (p == 0) {
        return HAL_GPIO_ReadPin(R0_GPIO_Port, R0_Pin);
    } else if (p == 1) {
        return HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin);
    } else {
        set_mux_addr(p - 2);
        return HAL_GPIO_ReadPin(RMUX_GPIO_Port, RMUX_Pin);
    }
    return 0;  // unreachable code
}

int get_mat2_pin(int p) {
    if (p < 0 || p > 7)
        return -1;
    return HAL_GPIO_ReadPin(cols_ports[p], cols_pins[p]);
}

void set_matrix_column_pin(int p, int s) {
    if (p < 0 || p > 7)
        return;
    HAL_GPIO_WritePin(cols_ports[p], cols_pins[p], s);
}

void init_keys() {
    for (int c = 0; c < N_HW_MAT_COLS; c++) {
        set_matrix_column_pin(c, GPIO_PIN_RESET);
    }
}

void read_keys(bool *S) {
    /***
     * WITH DIODES MATRIX
    */
    static int col_mapping[N_HW_MAT_COLS] = {0, 4, 3, 6, 5, 1, 7, 2};
    for (int c = 0; c < N_HW_MAT_COLS; c++) {
        set_matrix_column_pin(c, GPIO_PIN_SET);
        for (int r = 0; r < N_HW_MAT_ROWS; r++) {
            int v = get_matrix_row_pin(r);
#warning Remove this check
            if (v < 0) {
                Error_Handler();
            }
            // S[r]  = v; // for debug, single column only
            // S[c * N_HW_MAT_COLS + r] = v; // without mapping
            int pos                                      = (c * N_HW_MAT_COLS + r) % N_HW_MAT_COLS;  // -> you have to apply mapping to this
            int offset                                   = (c * N_HW_MAT_COLS + r) / N_HW_MAT_COLS;
            S[col_mapping[pos] * N_HW_MAT_COLS + offset] = v;
        }
        set_matrix_column_pin(c, GPIO_PIN_RESET);
    }

    /***
     * WITH MULTIPLEXERS
    */
    /* 
        // set_mux_addr(0);
        // return HAL_GPIO_ReadPin(MUX_D0_GPIO_Port, MUX_D0_Pin);
        for (int iaddr = 0; iaddr < (1 << N_MUX_CONTROL_PINS); iaddr++) {
            set_mux_addr(iaddr);
            if (!HAL_GPIO_ReadPin(MUX_D0_GPIO_Port, MUX_D0_Pin)) {
                nstate |= 1 << iaddr;
            }
        } 
    */
}
