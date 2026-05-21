#include "rc_sbus.h"
#include "drone_config.h"
#include "math_utils.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "RC_SBUS";

// SBUS frame: 25 bytes, 100kbps, 8E2, inverted
#define SBUS_FRAME_LEN      25
#define SBUS_HEADER         0x0F
#define SBUS_FOOTER         0x00
#define SBUS_CH_MIN         172
#define SBUS_CH_MAX         1811
#define SBUS_CH_MID         992
#define SBUS_UART_BAUD      100000

// RC Channel mapping (0-indexed, NAN = not mapped)
// Thay đổi nếu transmitter của bạn có mapping khác
#define RC_CH_ROLL          0    // CH1: Right stick horizontal
#define RC_CH_PITCH         1    // CH2: Right stick vertical
#define RC_CH_THROTTLE      2    // CH3: Left stick vertical
#define RC_CH_YAW           3    // CH4: Left stick horizontal
#define RC_CH_MODE          4    // CH5: Mode switch

static uint16_t s_channels[16] = {0};
static bool     s_frame_valid   = false;
static bool     s_initialized   = false;

// Calibration: zero (center/min) and max values per channel
static uint16_t s_ch_zero[16]; // center or min depending on channel
static uint16_t s_ch_max[16];

static void sbus_default_calibration(void) {
    for (int i = 0; i < 16; i++) {
        s_ch_zero[i] = SBUS_CH_MID;
        s_ch_max[i]  = SBUS_CH_MAX;
    }
    // Throttle: zero is at minimum (not center)
    s_ch_zero[RC_CH_THROTTLE] = SBUS_CH_MIN;
}

static bool sbus_parse_frame(const uint8_t *buf) {
    if (buf[0] != SBUS_HEADER) return false;
    // Decode 16 channels from 11-bit packed data
    s_channels[0]  = ((buf[1]       | buf[2]  << 8) & 0x07FF);
    s_channels[1]  = ((buf[2]  >> 3 | buf[3]  << 5) & 0x07FF);
    s_channels[2]  = ((buf[3]  >> 6 | buf[4]  << 2 | buf[5] << 10) & 0x07FF);
    s_channels[3]  = ((buf[5]  >> 1 | buf[6]  << 7) & 0x07FF);
    s_channels[4]  = ((buf[6]  >> 4 | buf[7]  << 4) & 0x07FF);
    s_channels[5]  = ((buf[7]  >> 7 | buf[8]  << 1 | buf[9] << 9) & 0x07FF);
    s_channels[6]  = ((buf[9]  >> 2 | buf[10] << 6) & 0x07FF);
    s_channels[7]  = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    s_channels[8]  = ((buf[12]      | buf[13] << 8) & 0x07FF);
    s_channels[9]  = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
    s_channels[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
    s_channels[11] = ((buf[16] >> 1 | buf[17] << 7) & 0x07FF);
    s_channels[12] = ((buf[17] >> 4 | buf[18] << 4) & 0x07FF);
    s_channels[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF);
    s_channels[14] = ((buf[20] >> 2 | buf[21] << 6) & 0x07FF);
    s_channels[15] = ((buf[21] >> 5 | buf[22] << 3) & 0x07FF);
    return true;
}

static float normalize_channel(int ch_idx, bool centered) {
    float zero = (float)s_ch_zero[ch_idx];
    float max  = (float)s_ch_max[ch_idx];
    float val  = (float)s_channels[ch_idx];
    if (centered) {
        // Centered: [zero..max] → [0..1], [min..zero] → [-1..0]
        return mapf(val, zero, max, 0.0f, 1.0f) * 2.0f - 1.0f;
    } else {
        // One-directional: [zero..max] → [0..1]
        return mapf(val, zero, max, 0.0f, 1.0f);
    }
}

void rc_sbus_init(void) {
    sbus_default_calibration();

    uart_config_t uart_cfg = {
        .baud_rate  = SBUS_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_EVEN,
        .stop_bits  = UART_STOP_BITS_2,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(BOARD_RC_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_RC_UART_NUM,
                                 UART_PIN_NO_CHANGE,  // TX not used
                                 BOARD_RC_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    // SBUS uses inverted logic - set inversion on UART RX
    ESP_ERROR_CHECK(uart_driver_install(BOARD_RC_UART_NUM,
                                        256, 0, 0, NULL, 0));
    uart_set_line_inverse(BOARD_RC_UART_NUM, UART_SIGNAL_RXD_INV);

    s_initialized = true;
    ESP_LOGI(TAG, "SBUS initialized on UART%d, GPIO%d",
             BOARD_RC_UART_NUM, BOARD_RC_RX_PIN);
}

bool rc_sbus_read(rc_input_t *out) {
    if (!s_initialized) { out->valid = false; return false; }

    uint8_t buf[SBUS_FRAME_LEN];
    // Drain and find valid frames
    size_t avail = 0;
    uart_get_buffered_data_len(BOARD_RC_UART_NUM, &avail);
    if (avail < SBUS_FRAME_LEN) { out->valid = false; return false; }

    // Read one frame
    int len = uart_read_bytes(BOARD_RC_UART_NUM, buf, SBUS_FRAME_LEN, 0);
    if (len < SBUS_FRAME_LEN || !sbus_parse_frame(buf)) {
        // Flush stale data and resync
        uart_flush_input(BOARD_RC_UART_NUM);
        out->valid = false;
        return false;
    }

    // Normalize channels
    out->roll     = clampf(normalize_channel(RC_CH_ROLL,     true),  -1, 1);
    out->pitch    = clampf(normalize_channel(RC_CH_PITCH,    true),  -1, 1);
    out->yaw      = clampf(normalize_channel(RC_CH_YAW,      true),  -1, 1);
    out->throttle = clampf(normalize_channel(RC_CH_THROTTLE, false),  0, 1);
    out->mode     = clampf(normalize_channel(RC_CH_MODE,     false),  0, 1);
    out->valid    = true;
    s_frame_valid = true;
    return true;
}

void rc_sbus_get_raw(uint16_t channels[16]) {
    memcpy(channels, s_channels, sizeof(s_channels));
}

void rc_sbus_print_calibration(void) {
    ESP_LOGI(TAG, "%-12s %-6s %-6s", "Control", "Zero", "Max");
    ESP_LOGI(TAG, "%-12s %-6d %-6d", "Roll",     s_ch_zero[RC_CH_ROLL],     s_ch_max[RC_CH_ROLL]);
    ESP_LOGI(TAG, "%-12s %-6d %-6d", "Pitch",    s_ch_zero[RC_CH_PITCH],    s_ch_max[RC_CH_PITCH]);
    ESP_LOGI(TAG, "%-12s %-6d %-6d", "Throttle", s_ch_zero[RC_CH_THROTTLE], s_ch_max[RC_CH_THROTTLE]);
    ESP_LOGI(TAG, "%-12s %-6d %-6d", "Yaw",      s_ch_zero[RC_CH_YAW],      s_ch_max[RC_CH_YAW]);
    ESP_LOGI(TAG, "%-12s %-6d %-6d", "Mode",     s_ch_zero[RC_CH_MODE],     s_ch_max[RC_CH_MODE]);
}
