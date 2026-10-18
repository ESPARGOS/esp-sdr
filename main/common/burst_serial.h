#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef enum { BURST_SERIAL_USB, BURST_SERIAL_UART, BURST_SERIAL_COUNT } burst_serial_port_t;

void burst_serial_init(void);
/* 1: complete line, -1: overlong line, 0: idle. Each port has its own parser.
 * A complete line selects the port for all subsequent replies and payloads.
 * BAUD? and BAUD <1000000|2000000> are consumed internally. Baud changes
 * last until reset; OK BAUD <rate> is sent at the previous rate. */
int burst_serial_poll_line(char *line, size_t capacity);
burst_serial_port_t burst_serial_port(void);
unsigned burst_serial_baud(void);
bool burst_serial_send(const void *data, size_t size);
/* Write only what fits now; never wait for the transport. */
size_t burst_serial_try_send(const void *data, size_t size);

/* Consume a stream stop request on the owning port only. */
bool burst_serial_stop_requested(void);

/* Read commands forever and pass each line to `handle`. One port at a time
 * owns the receiver: the other gets "ERR busy" until the owner sends RELEASE
 * or stays silent for five seconds. RELEASE itself is answered here. */
void burst_serial_serve(void (*handle)(char *line));
/* Optional work for burst_serial_serve(), such as a stand-alone display:
 * called with host_active=false while no port holds the lease, and with
 * true while one does, including once before each host command runs. */
void burst_serial_set_idle(void (*idle)(bool host_active));
/* Optional read-only commands, such as display buffer dumps: tried first on
 * every line; returns true if it answered. These neither take the lease nor
 * pause the idle work. */
void burst_serial_set_passive(bool (*passive)(const char *line));
