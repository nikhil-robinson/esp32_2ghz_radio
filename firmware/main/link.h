/*
 * Host link over native USB-Serial-JTAG and (optionally) UART0.
 *
 * Both ports are polled; a valid request frame selects that port for every
 * reply and leases it for LINK_LEASE_US. While leased, requests on the other
 * port are answered with ERR_BUSY.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_sdr.h"

typedef enum { LINK_PORT_USB = 0, LINK_PORT_UART = 1, LINK_PORT_COUNT } link_port_t;

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint32_t length;
    const uint8_t *payload;  /* valid until the next link_poll() */
} link_request_t;

void link_init(void);
/* Returns true when a request is ready. Never blocks. */
bool link_poll(link_request_t *request);
link_port_t link_port(void);
void link_release(void);
/* UART only. Caller has already acknowledged at the old rate. */
esp_err_t link_set_uart_baud(uint32_t baud);
bool link_uart_active(void);

/* Blocking writes with a size-dependent deadline; false on timeout. */
bool link_send(const void *data, size_t size);
bool link_send_frame(uint8_t type, uint8_t seq, const void *payload, size_t size);
/* Frame whose payload is `head` followed by `body` (no copy of the body). */
bool link_send_frame_split(uint8_t type, uint8_t seq, const void *head, size_t head_size,
                           const void *body, size_t body_size);

/* Sink for esp_sdr_run(): records go to the active port; any received byte
 * requests a stop. Call link_discard_input() after the run. */
const esp_sdr_sink_t *link_stream_sink(void);
/* Drop input that arrived during a stream (the stop request). */
void link_discard_input(void);
