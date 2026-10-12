#include "link.h"

#include <string.h>
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "frame.h"
#include "protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_ll.h"
#include "hal/usb_serial_jtag_ll.h"
#include "sdkconfig.h"
#include "soc/uart_struct.h"

#define USB_PACKET 64u
#define UART_CHUNK 128u
#define UART_RX_BUFFER 8192
#define LEASE_US 5000000
#define PARTIAL_FRAME_TIMEOUT_US 1000000
#define USB_SEND_TIMEOUT_US 3000000
#define UART_SEND_SLACK_US 2000000

#if CONFIG_ESP_SDR_UART_ENABLED
#define UART_PORT UART_NUM_0
static uint32_t uart_baud = CONFIG_ESP_SDR_UART_BAUD;
#endif

static link_port_t active = LINK_PORT_USB;
static int owner = -1;
static int64_t lease_until;
static unsigned next_poll;

static struct {
    frame_parser_t parser;
    int64_t last_byte;
    uint8_t stash[USB_PACKET];  /* bytes read but not yet parsed */
    uint8_t stash_len;
    uint8_t stash_pos;
} rx[LINK_PORT_COUNT];

void link_init(void)
{
    for (unsigned p = 0; p < LINK_PORT_COUNT; p++) {
        frame_parser_reset(&rx[p].parser);
    }
#if CONFIG_ESP_SDR_UART_ENABLED
    const uart_config_t config = {
        .baud_rate = (int)uart_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, CONFIG_ESP_SDR_UART_TX_PIN, CONFIG_ESP_SDR_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    /* Interrupt-driven RX buffers requests; TX writes the FIFO directly so
     * every wait has a deadline. */
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUFFER, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_flush_input(UART_PORT));
#endif
}

link_port_t link_port(void) { return active; }
bool link_uart_active(void) { return active == LINK_PORT_UART; }

void link_release(void)
{
    owner = -1;
}

static int read_port(link_port_t port, uint8_t *buffer, size_t size)
{
    if (port == LINK_PORT_UART) {
#if CONFIG_ESP_SDR_UART_ENABLED
        return uart_read_bytes(UART_PORT, buffer, size, 0);
#else
        return 0;
#endif
    }
    return (int)usb_serial_jtag_ll_read_rxfifo(buffer, size > USB_PACKET ? USB_PACKET : size);
}

static void send_ack(uint8_t seq, uint8_t request, uint8_t error)
{
    msg_ack_t ack = {.error = error, .request = request};
    link_send_frame(RSP_ACK, seq, &ack, sizeof(ack));
}

bool link_poll(link_request_t *request)
{
    int64_t now = esp_timer_get_time();
    if (owner >= 0 && now >= lease_until) {
        owner = -1;
    }
    for (unsigned j = 0; j < LINK_PORT_COUNT; j++) {
        link_port_t port = (link_port_t)((next_poll + j) % LINK_PORT_COUNT);
        typeof(rx[0]) *in = &rx[port];
        if (frame_parser_busy(&in->parser) && now - in->last_byte > PARTIAL_FRAME_TIMEOUT_US) {
            frame_parser_reset(&in->parser);
        }
        if (in->stash_pos == in->stash_len) {
            int n = read_port(port, in->stash, sizeof(in->stash));
            in->stash_len = (uint8_t)(n > 0 ? n : 0);
            in->stash_pos = 0;
        }
        while (in->stash_pos < in->stash_len) {
            in->last_byte = now;
            frame_event_t event = frame_parser_push(&in->parser, in->stash[in->stash_pos++]);
            if (event == FRAME_NONE || event == FRAME_BAD_CRC) {
                continue;  /* a corrupted request is ignored; the host times out and retries */
            }
            active = port;
            next_poll = (port + 1u) % LINK_PORT_COUNT;
            if (event == FRAME_TOO_LONG) {
                send_ack(0, 0, WIRE_ERR_BAD_LENGTH);
                continue;
            }
            if (owner >= 0 && owner != (int)port) {
                send_ack(in->parser.seq, in->parser.type, WIRE_ERR_BUSY);
                continue;
            }
            owner = port;
            lease_until = now + LEASE_US;
            request->type = in->parser.type;
            request->seq = in->parser.seq;
            request->length = in->parser.length;
            request->payload = in->parser.payload;
            return true;  /* remaining stashed bytes are parsed on the next call */
        }
    }
    return false;
}

static int64_t send_deadline(size_t size)
{
#if CONFIG_ESP_SDR_UART_ENABLED
    if (active == LINK_PORT_UART) {
        /* 8N1: ten wire bits per byte, plus slack for host scheduling. */
        return esp_timer_get_time() + UART_SEND_SLACK_US + (int64_t)size * 10000000 / uart_baud;
    }
#endif
    (void)size;
    return esp_timer_get_time() + USB_SEND_TIMEOUT_US;
}

bool link_send(const void *data, size_t size)
{
    const uint8_t *p = data;
    const size_t total = size;
    const int64_t deadline = send_deadline(size);
    while (size) {
        if (esp_timer_get_time() >= deadline) {
            return false;
        }
        int sent;
        if (active == LINK_PORT_UART) {
#if CONFIG_ESP_SDR_UART_ENABLED
            sent = uart_tx_chars(UART_PORT, (const char *)p, size > UART_CHUNK ? UART_CHUNK : size);
            if (sent < 0) {
                return false;
            }
            if (sent == 0) {
                vTaskDelay(1);
            }
#else
            return false;
#endif
        } else {
            if (!usb_serial_jtag_ll_txfifo_writable()) {
                continue;
            }
            sent = (int)usb_serial_jtag_ll_write_txfifo(p, size > USB_PACKET ? USB_PACKET : size);
            usb_serial_jtag_ll_txfifo_flush();
        }
        p += sent;
        size -= (size_t)sent;
    }
    /* A transfer ending on a full packet needs a zero-length packet to be
     * delivered promptly: flush once more after the FIFO drains. */
    if (active == LINK_PORT_USB && total && total % USB_PACKET == 0) {
        while (!usb_serial_jtag_ll_txfifo_writable()) {
            if (esp_timer_get_time() >= deadline) {
                return false;
            }
        }
        usb_serial_jtag_ll_txfifo_flush();
    }
    return true;
}

bool link_send_frame_split(uint8_t type, uint8_t seq, const void *head, size_t head_size,
                           const void *body, size_t body_size)
{
    uint8_t header[ESP_SDR_RECORD_HEADER_SIZE];
    uint32_t crc = frame_write_header(header, type, seq, (uint32_t)(head_size + body_size));
    crc = frame_crc32(crc, head, head_size);
    crc = frame_crc32(crc, body, body_size);
    return link_send(header, sizeof(header)) && link_send(head, head_size) && link_send(body, body_size) &&
           link_send(&crc, sizeof(crc));
}

bool link_send_frame(uint8_t type, uint8_t seq, const void *payload, size_t size)
{
    return link_send_frame_split(type, seq, payload, size, NULL, 0);
}

esp_err_t link_set_uart_baud(uint32_t baud)
{
#if CONFIG_ESP_SDR_UART_ENABLED
    if (active != LINK_PORT_UART) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(1000)), "link", "drain");
    ESP_RETURN_ON_ERROR(uart_set_baudrate(UART_PORT, baud), "link", "baud");
    uart_baud = baud;
    uart_flush_input(UART_PORT);
    frame_parser_reset(&rx[LINK_PORT_UART].parser);
    return ESP_OK;
#else
    (void)baud;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

/* ---- stream sink (interrupts masked; never blocks) -------------------- */

IRAM_ATTR static bool sink_stop_requested(void *ctx)
{
#if CONFIG_ESP_SDR_UART_ENABLED
    if (active == LINK_PORT_UART) {
        /* The UART ISR cannot run while a stream masks interrupts. */
        return uart_ll_get_rxfifo_len(&UART0) > 0;
    }
#endif
    return usb_serial_jtag_ll_rxfifo_data_available();
}

IRAM_ATTR static size_t sink_write(void *ctx, const uint8_t *data, size_t size)
{
#if CONFIG_ESP_SDR_UART_ENABLED
    if (active == LINK_PORT_UART) {
        /* Direct FIFO write: the UART driver's ISR cannot run while masked. */
        uint32_t room = uart_ll_get_txfifo_len(&UART0);
        uint32_t n = size < room ? (uint32_t)size : room;
        uart_ll_write_txfifo(&UART0, data, n);
        return n;
    }
#endif
    if (!usb_serial_jtag_ll_txfifo_writable()) {
        return 0;
    }
    int written = (int)usb_serial_jtag_ll_write_txfifo(data, (uint32_t)size);
    usb_serial_jtag_ll_txfifo_flush();
    return written > 0 ? (size_t)written : 0;
}

static const esp_sdr_sink_t stream_sink = {
    .write = sink_write,
    .stop_requested = sink_stop_requested,
    .ctx = NULL,
};

bool link_stop_requested(void)
{
    return sink_stop_requested(NULL);
}

const esp_sdr_sink_t *link_stream_sink(void)
{
    return &stream_sink;
}

void link_discard_input(void)
{
    uint8_t scratch[USB_PACKET];
#if CONFIG_ESP_SDR_UART_ENABLED
    if (active == LINK_PORT_UART) {
        while (uart_ll_get_rxfifo_len(&UART0) > 0) {
            uart_ll_read_rxfifo(&UART0, scratch, 1);
        }
        uart_flush_input(UART_PORT);
        rx[LINK_PORT_UART].stash_len = rx[LINK_PORT_UART].stash_pos = 0;
        frame_parser_reset(&rx[LINK_PORT_UART].parser);
        return;
    }
#endif
    while (usb_serial_jtag_ll_rxfifo_data_available()) {
        usb_serial_jtag_ll_read_rxfifo(scratch, sizeof(scratch));
    }
    rx[LINK_PORT_USB].stash_len = rx[LINK_PORT_USB].stash_pos = 0;
    frame_parser_reset(&rx[LINK_PORT_USB].parser);
}
