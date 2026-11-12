/* C5 bank ownership covers 128 KiB, although the dump writes only 64 KiB.
 * Bank 2's upper half holds live heap/ROM data and may hold the command task's
 * stack. Run the interrupt-masked ring engine on LP SRAM, passing only config
 * and result objects below the RF aperture. No caller-owned pointer may be
 * dereferenced until the engine restores bank ownership and returns. */
#include "ring_capture.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static RTC_NOINIT_ATTR StackType_t worker_stack[8192 / sizeof(StackType_t)];
static StaticTask_t worker_tcb;
static TaskHandle_t worker, caller;
static ring_config_t config;
static ring_result_t result;

static void capture_worker(void *arg) {
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint32_t marker;
        if ((uintptr_t)&marker < 0x50000000u || (uintptr_t)&marker >= 0x50004000u) {
            memset(&result, 0, sizeof(result));
            result.status = RING_FAIL_ARG;
            result.detail = 6;
        } else {
            ring_capture_run(&config, &result);
        }
        xTaskNotifyGive(caller);
    }
}

void c5_ring_capture_run(const ring_config_t *cfg, ring_result_t *out) {
    if (!worker)
        worker = xTaskCreateStatic(capture_worker, "c5_capture", sizeof(worker_stack), NULL,
                                   20, worker_stack, &worker_tcb);
    if (!worker) {
        memset(out, 0, sizeof(*out));
        out->status = RING_FAIL_ARG;
        out->detail = 5;
        return;
    }
    /* The shared command server serializes capture requests. */
    config = *cfg;
    caller = xTaskGetCurrentTaskHandle();
    xTaskNotifyGive(worker);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    *out = result;
}
