/* Continuous S31 modem diagnostic bus -> GPIO matrix loopback -> PARLIO GDMA.
 * No CPU sampling loop, TCM refill windows, or TX implementation. */
#include "driver/gpio.h"
#include "driver/parlio_rx.h"
#include "esp_clk_tree.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_ipc_isr.h"
#include "esp_log.h"
#include "esp_phy_cert_test.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/parlio_ll.h"
#include "heap_memory_layout.h"
#include "modem/modem_widgets_reg.h"
#include "nvs_flash.h"
#include "rx_bandwidth.h"
#include "soc/gpio_sig_map.h"
#include "soc/hp_system_reg.h"
#include "stream.h"
#include <string.h>

/* Opening the diagnostic producer briefly also enables the TCM writer.
 * Reserve its entire upper aperture; the ongoing sample path is independent. */
SOC_RESERVE_MEMORY_REGION(0x2f060000, 0x2f07f170, stream_rf_dump);
#define DMA_NODE 4032u
#define DMA_NODES 8u
#define DMA_BYTES (DMA_NODE * DMA_NODES)
static const int pins[16] = {20, 21, 22, 23, 24, 25, 33, 34, 36, 37, 38, 39, 40, 42, 43, 44};
static parlio_rx_unit_handle_t unit;
static parlio_rx_delimiter_handle_t delimiter;
static uint8_t *dma_buffer;
static TaskHandle_t capture_task;
static QueueHandle_t commands, replies;
static portMUX_TYPE gate_mux = portMUX_INITIALIZER_UNLOCKED;
static bool running;
static uint32_t event_head;
static struct {
    const uint8_t *data;
    unsigned bytes;
    uint64_t sample;
} events[DMA_NODES];
static uint64_t start_us, source_samples;
void receiver_transport_lost(stream_owner_t owner) {
    uint32_t expected = owner;
    __atomic_compare_exchange_n(&stream_owner, &expected, STREAM_IDLE, false, __ATOMIC_ACQ_REL,
                                __ATOMIC_ACQUIRE);
    if (capture_task)
        xTaskNotifyGive(capture_task);
}
stream_packet_t *stream_ring;
uint32_t stream_head, stream_tail, stream_epoch, stream_owner;
receiver_config_t receiver_config = {2412000000u, 16000000u, 40, 0, 1, 0};
unsigned receiver_gain_max;
typedef struct {
    receiver_config_t config;
    stream_owner_t owner;
} command_t;
extern void phy_chip_set_chan(unsigned, unsigned);
extern void phy_set_freq(unsigned, int);
extern void phy_pbus_workmode(void);
extern void phy_pbus_debugmode(void);
extern void phy_pbus_xpd_tx_off(void);
extern void phy_pbus_xpd_rx_on(int);
extern void phy_set_rxclk_en(int);
extern void phy_set_txclk_en(unsigned);
extern void phy_loopback_mode_en(unsigned);
extern void phy_bb_bss_cbw40_dig(unsigned);
extern void phy_bb_cbw_chan_cfg(unsigned);
extern void phy_wifi_fbw_sel(unsigned);
extern void phy_force_rx_gain(unsigned, unsigned);
extern void phy_set_rx_gain_table(unsigned, unsigned);
extern void phy_rfrx_sat_rst(unsigned);
extern unsigned phy_i2c_readReg(unsigned, unsigned, unsigned);
extern void phy_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
extern unsigned char phy_param[];
extern void burst_gain_mirror(int);
#include "tuning.h"
int cmd_parse(char *cmd, char *name, int *argc, char **argv) {
    (void)cmd;
    (void)name;
    (void)argc;
    (void)argv;
    return -1;
}
static void prepare_rx(void) {
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
}
static unsigned gain_init_default, gain_threshold_default;
static bool gain_defaults_saved;
static void radio_configure(const receiver_config_t *c) {
    /* Remove manual overrides before the PHY configures the new channel. */
    phy_set_txclk_en(1);
    burst_gain_mirror(-1);
    phy_set_txclk_en(0);
    if (gain_defaults_saved) {
        REG_WRITE(0x20107094, gain_init_default);
        REG_WRITE(0x2010713c, gain_threshold_default);
    }
    s31_tune(c->frequency_hz / 1000000u);
    phy_loopback_mode_en(0);
    phy_bb_bss_cbw40_dig(0);
    phy_bb_cbw_chan_cfg(0);
    phy_wifi_fbw_sel(0);
    prepare_rx();
    phy_pbus_debugmode();
    *(volatile unsigned *)(phy_param + 164) &= ~0x200u;
    phy_set_rx_gain_table(c->frequency_hz / 1000000u, 0);
    receiver_gain_max = (REG_READ(0x2010702c) >> 8) & 127;
    gain_init_default = REG_READ(0x20107094);
    gain_threshold_default = REG_READ(0x2010713c);
    gain_defaults_saved = true;
    phy_set_txclk_en(1);
    burst_gain_mirror(c->agc ? -1 : (int)c->gain);
    phy_set_txclk_en(0);
    prepare_rx();
    phy_rfrx_sat_rst(c->agc ? 1 : 0);
    if (!c->agc) {
        REG_WRITE(0x20107094, (gain_init_default & ~0x1fcu) | (c->gain << 2));
        REG_WRITE(0x2010713c, (gain_threshold_default & ~0x01fc0000u) | (c->gain << 18));
    }
    phy_force_rx_gain(c->agc ? 0 : 1, c->agc ? 0 : c->gain);
    unsigned cap = rx_bandwidth_dcap(receiver_bandwidth(c) / 1000000u);
    for (unsigned j = 4; j <= 5; j++)
        phy_i2c_writeReg(0x67, 1, j, (phy_i2c_readReg(0x67, 1, j) & ~63u) | cap);
}
static void IRAM_ATTR diagnostic_start(void) {
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&gate_mux);
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0xff000000u);
    __asm__ volatile("fence" ::: "memory");
    REG_CLR_BIT(0x201008cc, 0x00780000);
    REG_WRITE(0x201070b8, (REG_READ(0x201070b8) & ~7u) | 1u);
    REG_SET_BIT(0x20100800, 4);
    REG_WRITE(0x20109c04, UINT32_MAX);
    REG_CLR_BIT(0x2010900c, BIT(31));
    REG_SET_BIT(HP_SYSTEM_TCM_RAM_PWR_CTRL0_REG, HP_SYSTEM_REG_HP_SYSTEM_TCM_CLK_FORCE_ON);
    REG_SET_BIT(0x20109c04, BIT(31) | BIT(21));
    REG_SET_BIT(0x20109c08, BIT(31));
    REG_SET_BIT(0x20109c14, 0xe400);
    REG_WRITE(0x20109c0c, (REG_READ(0x20109c0c) & ~0xff00f000u) | 0x44004000u);
    REG_SET_BIT(0x20109c10, BIT(31));
    REG_CLR_BIT(0x20109c10, BIT(31));
    REG_WRITE(0x20109008, (REG_READ(0x20109008) & ~0x01fe0000u) | (3u << 17));
    REG_WRITE(0x20109018, (REG_READ(0x20109018) & ~0xffffffu) | 24 | (25 << 6) | (26 << 12) |
                              (27 << 18) | BIT(24));
    unsigned ctrl = (REG_READ(0x20109004) & ~0x803fffffu) | 16383 | BIT(17);
    REG_WRITE(0x20109004, ctrl | BIT(18));
    REG_WRITE(0x20109004, ctrl);
    REG_SET_BIT(HP_SYSTEM_CLK_EN_REG, HP_SYSTEM_REG_CLK_EN);
    REG_SET_BIT(HP_SYSTEM_PROBEA_CTRL_REG, HP_SYSTEM_REG_PROBE_GLOBAL_EN);
    REG_SET_BIT(MODEM_WIDGETS_CLK_CONF_REG, MODEM_WIDGETS_CLK_EN);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG, 0x32f2);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG, MODEM_WIDGETS_DIAG_BUS_FIX_LOW_EN | (22u << 5));
    REG_WRITE(0x20107c04, REG_READ(0x20107c04) & ~(0xffu | BIT(30)));
    REG_WRITE(HP_SYSTEM_MODEM_DIAG_EN_REG, UINT32_MAX);
    REG_WRITE(0x20109004, ctrl | BIT(31));
    for (volatile unsigned j = 0; j < 64; j++)
        __asm__ volatile("nop");
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0);
    __asm__ volatile("fence" ::: "memory");
    taskEXIT_CRITICAL(&gate_mux);
    esp_ipc_isr_release_other_cpu();
}
static bool IRAM_ATTR dma_done(parlio_rx_unit_handle_t u, const parlio_rx_event_data_t *e,
                               void *arg) {
    (void)u;
    (void)arg;
    uint32_t h = stream_load(&event_head);
    events[h % DMA_NODES].data = e->data;
    events[h % DMA_NODES].bytes = e->recv_bytes;
    events[h % DMA_NODES].sample = source_samples;
    source_samples += e->recv_bytes / 2;
    stream_store(&event_head, h + 1);
    BaseType_t woke = pdFALSE;
    vTaskNotifyGiveFromISR(capture_task, &woke);
    return woke == pdTRUE;
}
static bool dc_measure(float mean[2]) {
    source_samples = 0;
    stream_store(&event_head, 0);
    ESP_ERROR_CHECK(parlio_rx_unit_enable(unit, true));
    ESP_ERROR_CHECK(parlio_rx_soft_delimiter_start_stop(unit, delimiter, true));
    parlio_receive_config_t rx = {.delimiter = delimiter, .flags.partial_rx_en = true};
    ESP_ERROR_CHECK(parlio_rx_unit_receive(unit, dma_buffer, DMA_BYTES, &rx));
    unsigned tail = 0, count = 0;
    int64_t sum_i = 0, sum_q = 0;
    int64_t settled = esp_timer_get_time() + 50000;
    int64_t deadline = settled + 50000;
    while (count < 16384 && esp_timer_get_time() < deadline) {
        unsigned h = stream_load(&event_head);
        if (h == tail) { ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1)); continue; }
        if (h - tail >= DMA_NODES - 1) tail = h - 1;
        unsigned bytes = events[tail % DMA_NODES].bytes;
        const int8_t *data = (const int8_t *)events[tail % DMA_NODES].data;
        int si = 0, sq = 0; unsigned n = 0;
        /* Sample every pair: periodic sparse taps can alias a test tone into
         * the DC estimate. Full sums also average packet/clock artifacts. */
        if (bytes <= 4092 && !(bytes & 1))
            for (unsigned j = 0; j < bytes; j += 2) { si += data[j]; sq += data[j+1]; n++; }
        if (esp_timer_get_time() >= settled && stream_load(&event_head) - tail < DMA_NODES - 1) {
            sum_i += si; sum_q += sq; count += n;
        }
        tail++;
    }
    ESP_ERROR_CHECK(parlio_rx_soft_delimiter_start_stop(unit, delimiter, false));
    ESP_ERROR_CHECK(parlio_rx_unit_disable(unit));
    if (count < 16384) return false;
    mean[0] = (float)sum_i / count; mean[1] = (float)sum_q / count;
    return true;
}
static void capture_loop(void *arg) {
    (void)arg;
    unsigned tail = 0, partial = 0;
    uint64_t consumed = 0;
    command_t cmd;
    for (;;) {
        bool requested = xQueueReceive(commands, &cmd, 0) == pdTRUE;
        if (requested || (running && rx_recalibration_stale())) {
            if (!requested) {
                cmd.config = receiver_config;
                cmd.owner = stream_load(&stream_owner);
            }
            /* Restart after tracking with a new epoch, discarding old DMA
             * data. Never recalibrate while acquisition is running. */
            stream_store(&stream_owner, STREAM_IDLE);
            if (running) {
                parlio_rx_soft_delimiter_start_stop(unit, delimiter, false);
                ESP_ERROR_CHECK(parlio_rx_unit_disable(unit));
                running = false;
            }
            receiver_config = cmd.config;
            radio_configure(&cmd.config);
            diagnostic_start();
            uint32_t hz = 0;
            ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz(
                PARLIO_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &hz));
            hal_utils_clk_info_t info = {.src_freq_hz = hz,
                                         .exp_freq_hz = cmd.config.rate,
                                         .max_integ = PARLIO_LL_RX_MAX_CLK_INT_DIV,
                                         .min_integ = 1,
                                         .max_fract = PARLIO_LL_RX_MAX_CLK_FRACT_DIV};
            hal_utils_clk_div_t div = {.integer = 1};
            hal_utils_calc_clk_div_frac_accurate(&info, &div);
            parlio_ll_rx_set_clock_div(&PARL_IO, &div);
            parlio_ll_rx_update_config(&PARL_IO);
            receiver_dc_steps = 0;
            memset(receiver_dc_jacobian, 0, sizeof(receiver_dc_jacobian));
            memset(receiver_dc_before, 0, sizeof(receiver_dc_before));
            memset(receiver_dc_after, 0, sizeof(receiver_dc_after));
            if (cmd.owner != STREAM_IDLE && cmd.config.dc_correction && !cmd.config.agc)
                receiver_dc_calibrate(dc_measure);
            tail = 0;
            partial = 0;
            source_samples = consumed = 0;
            stream_store(&event_head, 0);
            stream_store(&stream_epoch, stream_load(&stream_epoch) + 1);
            if (cmd.owner != STREAM_IDLE) {
                start_us = esp_timer_get_time();
                ESP_ERROR_CHECK(parlio_rx_unit_enable(unit, true));
                ESP_ERROR_CHECK(parlio_rx_soft_delimiter_start_stop(unit, delimiter, true));
                parlio_receive_config_t rx = {.delimiter = delimiter, .flags.partial_rx_en = true};
                ESP_ERROR_CHECK(parlio_rx_unit_receive(unit, dma_buffer, DMA_BYTES, &rx));
                running = true;
                stream_store(&stream_owner, cmd.owner);
            }
            esp_err_t ok = ESP_OK;
            if (requested) xQueueSend(replies, &ok, portMAX_DELAY);
        }
        unsigned owner = stream_load(&stream_owner);
        if (running && (owner == STREAM_IDLE || (owner == STREAM_USB && !usb_ready()))) {
            parlio_rx_soft_delimiter_start_stop(unit, delimiter, false);
            ESP_ERROR_CHECK(parlio_rx_unit_disable(unit));
            running = false;
            partial = 0;
            stream_store(&stream_owner, STREAM_IDLE);
        }
        unsigned h = stream_load(&event_head);
        if (!running || h == tail) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
            continue;
        }
        if (h - tail >= DMA_NODES - 1) {
            /* The DMA producer can overwrite old nodes. Resume at a recent
             * complete node and preserve the absolute sample discontinuity. */
            tail = h - 1;
            uint64_t next = events[tail % DMA_NODES].sample;
            stream_record_drop(next - consumed + partial / 2, STREAM_DROP_DMA);
            consumed = next;
            partial = 0;
        }
        const uint8_t *data = events[tail % DMA_NODES].data;
        unsigned bytes = events[tail % DMA_NODES].bytes;
        uint64_t sample = events[tail % DMA_NODES].sample;
        uint32_t out = stream_load(&stream_head), read = stream_load(&stream_tail);
        /* An unfinished packet occupies a slot too. Account for the actual
         * descriptor length rather than assuming a fixed packet count. */
        unsigned slots = (partial + bytes + STREAM_PAYLOAD - 1) / STREAM_PAYLOAD;
        if (bytes > 4092 || (bytes & 1) || slots > STREAM_RING ||
            out - read > STREAM_RING - slots) {
            stream_record_drop(bytes / 2 + partial / 2, STREAM_DROP_RING);
            partial = 0;
            consumed = sample + bytes / 2;
            tail++;
            continue;
        }
        uint32_t first_out = out;
        unsigned copied = 0, initial_partial = partial;
        while (copied < bytes) {
            stream_packet_t *p = &stream_ring[out % STREAM_RING];
            if (!partial) {
                memcpy(p->magic, "ESR2", 4);
                p->version = STREAM_VERSION;
                p->bits = 8;
                p->epoch = stream_load(&stream_epoch);
                p->rate = cmd.config.rate;
                p->sample = sample + copied / 2;
                /* All supported clocks are integer MHz. Divide first so the
                 * timestamp stays valid beyond multi-day acquisitions. */
                p->time_us = start_us + p->sample / (p->rate / 1000000u);
            }
            unsigned n = STREAM_PAYLOAD - partial;
            if (n > bytes - copied)
                n = bytes - copied;
            memcpy(p->iq + partial, data + copied, n);
            partial += n;
            copied += n;
            if (partial == STREAM_PAYLOAD) {
                out++;
                partial = 0;
            }
        }
        if (stream_load(&event_head) - tail >= DMA_NODES - 1) {
            stream_record_drop(bytes / 2 + initial_partial / 2, STREAM_DROP_DMA);
            partial = 0;
            out = first_out;
        }
        stream_store(&stream_head, out);
        if (out != first_out)
            stream_wake_sender();
        consumed = sample + bytes / 2;
        tail++;
    }
}
void receiver_init(void) {
    stream_ring = heap_caps_aligned_alloc(64, STREAM_RING * sizeof(*stream_ring),
                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    dma_buffer = heap_caps_aligned_alloc(64, DMA_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    assert(stream_ring && dma_buffer);
    uint64_t mask = 0;
    for (unsigned i = 0; i < 16; i++)
        mask |= 1ull << pins[i];
    gpio_config_t gp = {.pin_bit_mask = mask, .mode = GPIO_MODE_INPUT_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&gp));
    parlio_rx_unit_config_t pc = {.trans_queue_depth = 1,
                                  .max_recv_size = DMA_BYTES,
                                  .dma_burst_size = 64,
                                  .data_width = 16,
                                  .clk_src = PARLIO_CLK_SRC_DEFAULT,
                                  .exp_clk_freq_hz = 16000000,
                                  .clk_in_gpio_num = -1,
                                  .clk_out_gpio_num = -1,
                                  .valid_gpio_num = -1};
    for (unsigned bit = 0; bit < 16; bit++)
        pc.data_gpio_nums[bit] = pins[bit ^ 8];
    ESP_ERROR_CHECK(parlio_new_rx_unit(&pc, &unit));
    for (unsigned bit = 0; bit < 16; bit++) {
        unsigned signal = bit < 6    ? HP_PROBE_TOP_OUT0_IDX + bit + 2
                          : bit < 8  ? HP_PROBE_TOP_OUT8_IDX + bit - 6
                          : bit < 12 ? HP_PROBE_TOP_OUT8_IDX + bit - 4
                                     : LCD_DATA_OUT_PAD_OUT0_IDX + bit - 12;
        esp_rom_gpio_connect_out_signal(pins[bit], signal, false, false);
    }
    parlio_rx_soft_delimiter_config_t dc = {.sample_edge = PARLIO_SAMPLE_EDGE_POS,
                                            .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
                                            .eof_data_len = DMA_BYTES};
    ESP_ERROR_CHECK(parlio_new_rx_soft_delimiter(&dc, &delimiter));
    parlio_rx_event_callbacks_t cb = {.on_partial_receive = dma_done};
    ESP_ERROR_CHECK(parlio_rx_unit_register_event_callbacks(unit, &cb, NULL));
    wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_power_domain_on();
    esp_phy_rftest_config(1);
    ESP_ERROR_CHECK(esp_wifi_init(&wc));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    commands = xQueueCreate(1, sizeof(command_t));
    replies = xQueueCreate(1, sizeof(esp_err_t));
    assert(commands && replies);
    BaseType_t created =
        xTaskCreatePinnedToCore(capture_loop, "capture", 6144, NULL, 22, &capture_task, 1);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    receiver_apply(&receiver_config, STREAM_IDLE);
}
esp_err_t receiver_apply(const receiver_config_t *c, stream_owner_t owner) {
    command_t cmd = {*c, owner};
    esp_err_t reply;
    xQueueSend(commands, &cmd, portMAX_DELAY);
    xTaskNotifyGive(capture_task);
    xQueueReceive(replies, &reply, portMAX_DELAY);
    return reply;
}
void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    receiver_init();
    control_init();
    usb_init();
    network_init();
    /* Drain the bounded sample ring ahead of USB/HTTP control callbacks on
     * either transport. The sender sleeps whenever the producer catches up. */
    BaseType_t created = xTaskCreatePinnedToCore(stream_sender, "sender", 6144, NULL, 24, NULL, 0);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
