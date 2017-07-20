#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "lmic.h"

const lmic_pinmap_t pins = {
    .nss = 22,
    .rst = 19,
    .dio = {23, 18, 5},
    .spi = {4, 2, 17},
};

static const char *TAG = "LMIC_HAL";

spi_device_handle_t spi_handle;

static void io_init () {

  ESP_LOGI(TAG, "Starting IO initialization");
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1<<pins.nss) | (1<<pins.rst);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1<<pins.dio[0]) | (1<<pins.dio[1]) | (1<<pins.dio[2]);
    gpio_config(&io_conf);
  ESP_LOGE(TAG, "Ending IO initialization");

}

static void spi_init () {
    esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num = pins.spi[0],
        .mosi_io_num = pins.spi[1],
        .sclk_io_num = pins.spi[2],
        .quadwp_io_num = -1, // not used here, hence is equal to -1
        .quadhd_io_num = -1, // same here
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 10000000,
        /*.mode = 0, // clock mode CPOL 0 and CPHA 0, which corresponds to mode0*/
        .mode = 1,
        .spics_io_num = -1,
        .queue_size = 7,
    };

    ESP_LOGE(TAG, "Starting SPI initialization");
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret == ESP_OK);


    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle);
    assert(ret == ESP_OK);

}

void hal_pin_nss (u1_t val) {
    gpio_set_level(pins.nss, val);
}


void hal_pin_rxtx (u1_t val) {
    // unused
}


void hal_pin_rst (u1_t val) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1<<pins.rst);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    if (val == 2) { // val == 2, floating state
        io_conf.mode = GPIO_MODE_INPUT;
        gpio_config(&io_conf);
    }
    else { // the pin is driven, val 0 or 1
        io_conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io_conf);
        gpio_set_level(pins.rst, val);
    }
}

bool dio_states[3];

// check IO and handle IRQs
void hal_io_check() {
    if (dio_states[0] != gpio_get_level(pins.dio[0])) {
        dio_states[0] = !dio_states[0];
        if (dio_states[0]) {
            ESP_LOGI(TAG, "Fired IRQ0\n");
            radio_irq_handler(0);
        }
    }

    if (dio_states[1] != gpio_get_level(pins.dio[1])) {
        dio_states[1] = !dio_states[1];
        if (dio_states[1]) {
            ESP_LOGI(TAG, "Fired IRQ1\n");
            radio_irq_handler(1);
        }
    }

    if (dio_states[2] != gpio_get_level(pins.dio[2])) {
        dio_states[2] = !dio_states[2];
        if (dio_states[2]) {
            ESP_LOGI(TAG, "Fired IRQ2\n");
            radio_irq_handler(2);
        }
    }
}


u1_t hal_spi (u1_t outval) {
    uint8_t rx_data = 0;
    spi_transaction_t spi_tr;

    memset(&spi_tr, 0, sizeof(spi_tr));

    spi_tr.length = 8;
    spi_tr.rxlength = 8;
    spi_tr.tx_buffer = &outval;
    spi_tr.rx_buffer = &rx_data;
    ESP_LOGE("Prout", "blewur");

    esp_err_t ret = spi_device_transmit(spi_handle, &spi_tr);
    ESP_LOGE("Prout", "dfjglskdf");
    assert(ret == ESP_OK);

    return (u1_t) rx_data;
}

static void time_init () {
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_1;

    timer_config_t timer_cfg;
    timer_cfg.alarm_en = 0;
    timer_cfg.auto_reload = 0;
    timer_cfg.counter_dir = TIMER_COUNT_UP;
    timer_cfg.divider = 120;
    timer_cfg.intr_type = 0;
    timer_cfg.counter_en = TIMER_PAUSE;

    timer_init(timer_group, timer_idx, &timer_cfg);
    timer_pause(timer_group, timer_idx);
    timer_set_counter_value(timer_group, timer_idx, 0x0);
    timer_start(timer_group, timer_idx);
}

u4_t hal_ticks () {
    uint64_t val;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_1, &val);
    printf("Getting some ticks\n");
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return (uint32_t) val;
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}


void hal_waitUntil (u4_t time) {

    ESP_LOGI(TAG, "Wait until");
    s4_t delta = delta_time(time);

    while(delta > 2000){
        vTaskDelay(1 / portTICK_PERIOD_MS);
        delta -= 1000;
    } if(delta > 0){
        vTaskDelay(delta / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Done waiting until");
}

u1_t hal_checkTimer (u4_t time) {
    return 1;
}

int enabled_irq = 1;

void hal_disableIRQs () {
    if(enabled_irq){
        taskDISABLE_INTERRUPTS();
        enabled_irq = 0;
    }
}

void hal_enableIRQs () {
    if(!enabled_irq){
        taskENABLE_INTERRUPTS();
        hal_io_check();
        enabled_irq = 1;
    }
}

void hal_failed () {
    hal_disableIRQs();
    hal_sleep();
    while(1);
}

void hal_sleep () {
}

void hal_init () {
    io_init();
    spi_init();
    time_init();
}
