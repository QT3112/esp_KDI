#include "battery.h"
#include "drone_config.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"

static const char *TAG = "BATTERY";

static adc_oneshot_unit_handle_t s_adc_handle;
static float s_voltage = 0.0f;

void battery_init(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,    // 0~3.3V range
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle,
                                                BOARD_VBAT_ADC_CHANNEL,
                                                &ch_cfg));
    ESP_LOGI(TAG, "Battery ADC initialized on channel %d", BOARD_VBAT_ADC_CHANNEL);
}

static float read_raw_voltage(void) {
    int32_t sum = 0;
    int raw = 0;
    for (int i = 0; i < BOARD_VBAT_ADC_SAMPLES; i++) {
        adc_oneshot_read(s_adc_handle, BOARD_VBAT_ADC_CHANNEL, &raw);
        sum += raw;
    }
    // Convert ADC raw to mV (12-bit ADC, 3300mV full scale at 12dB atten)
    float adc_mv = (sum / (float)BOARD_VBAT_ADC_SAMPLES) / 4095.0f * 3300.0f;
    // Apply voltage divider ratio
    return (adc_mv / 1000.0f) * BOARD_VBAT_DIVIDER;
}

void battery_update(void) {
    s_voltage = read_raw_voltage();
}

float battery_get_voltage(void) {
    return s_voltage;
}
