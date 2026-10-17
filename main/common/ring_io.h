/* Nonblocking transport access while capture masks interrupts. UART receive
 * must read hardware directly because its normal ISR cannot fill the driver
 * queue during a ring run. Ordinary commands retain the shared lease parser. */
#include "burst_serial.h"
#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "soc/uart_struct.h"
#include "soc/soc_caps.h"
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#include "hal/usb_serial_jtag_ll.h"
#endif
static bool ring_input_available(void) {
    if (burst_serial_port() == BURST_SERIAL_UART) return uart_ll_get_rxfifo_len(&UART0) > 0;
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    return usb_serial_jtag_ll_rxfifo_data_available();
#else
    return false;
#endif
}
static int ring_read_byte(uint8_t *b) {
    if (burst_serial_port() == BURST_SERIAL_UART) {
        if (!uart_ll_get_rxfifo_len(&UART0)) return 0;
        uart_ll_read_rxfifo(&UART0, b, 1);
        return 1;
    }
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    return usb_serial_jtag_ll_read_rxfifo(b, 1);
#else
    return 0;
#endif
}
/* An on-device consumer (stand-alone display) takes the stream instead of
 * the serial port; it is set only around its own runs. */
#if CONFIG_IDF_TARGET_ESP32S3
#include "ring_capture.h"
#define RING_LOCAL_SINK ring_capture_local_sink
#else
#define RING_LOCAL_SINK ((void (*)(const uint8_t *, unsigned))0)
#endif
static int ring_write(const uint8_t *p, unsigned n) {
    if (RING_LOCAL_SINK) {
        RING_LOCAL_SINK(p, n);
        return (int)n;
    }
    if (burst_serial_port() == BURST_SERIAL_UART) return uart_tx_chars(UART_NUM_0, (const char *)p, n);
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    if (!usb_serial_jtag_ll_txfifo_writable()) return 0;
#if CONFIG_IDF_TARGET_ESP32S3
    /* The ring is the only USB writer and flushes every packet, so a writable
     * FIFO is empty and holds the caller's <= 64 bytes. Skip the generic
     * per-byte FIFO status read (from inlanger/esp-sdr). */
    for (unsigned i = 0; i < n; i++) USB_SERIAL_JTAG.ep1.rdwr_byte = p[i];
    int written = (int)n;
#else
    int written = usb_serial_jtag_ll_write_txfifo(p, n);
#endif
    usb_serial_jtag_ll_txfifo_flush();
    return written;
#else
    return 0;
#endif
}
