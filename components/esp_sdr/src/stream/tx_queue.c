/*
 * Lock-free byte FIFO between the stream processors and the application sink.
 * head/tail are free-running counters; one producer core per run.
 */
#include <string.h>
#include "stream.h"

#define SINK_CHUNK 64u           /* one USB full-speed packet */
#define SINK_CHUNKS_PER_PUMP 4u  /* bounded work per pump between ring polls */
_Static_assert((TX_QUEUE_SIZE & (TX_QUEUE_SIZE - 1u)) == 0, "queue size must be a power of two");

static uint8_t queue[TX_QUEUE_SIZE];
static volatile uint32_t head;
static volatile uint32_t tail;

void tx_queue_reset(void)
{
    head = tail = 0;
}

STREAM_HOT bool tx_queue_push(const void *data, uint32_t size)
{
    uint32_t h = head;
    if (TX_QUEUE_SIZE - (h - tail) < size) {
        return false;
    }
    uint32_t offset = h & (TX_QUEUE_SIZE - 1u), first = TX_QUEUE_SIZE - offset;
    if (first > size) {
        first = size;
    }
    memcpy(queue + offset, data, first);
    memcpy(queue, (const uint8_t *)data + first, size - first);
    head = h + size;
    return true;
}

STREAM_HOT bool tx_queue_push_from_worker(const void *data, uint32_t size)
{
    uint32_t h = head;
    if (TX_QUEUE_SIZE - (h - tail) < size) {
        return false;
    }
    /* Byte loop: core 1 must not call flash-resident memcpy. */
    const uint8_t *src = data;
    for (uint32_t i = 0; i < size; i++) {
        queue[(h + i) & (TX_QUEUE_SIZE - 1u)] = src[i];
    }
    MEMORY_BARRIER();
    head = h + size;
    return true;
}

STREAM_HOT void tx_queue_pump(void)
{
    const esp_sdr_sink_t *sink = g_stream.sink;
    /* Several chunks while the sink keeps accepting them; bounded so a ring
     * poll is never delayed by more than SINK_CHUNKS_PER_PUMP chunks. */
    for (unsigned chunk = 0; chunk < SINK_CHUNKS_PER_PUMP; chunk++) {
        uint32_t used = head - tail;
        if (!used) {
            return;
        }
        uint32_t offset = tail & (TX_QUEUE_SIZE - 1u), n = TX_QUEUE_SIZE - offset;
        if (n > used) {
            n = used;
        }
        if (n > SINK_CHUNK) {
            n = SINK_CHUNK;
        }
        if (!sink || !sink->write) {
            tail += n;  /* no sink: discard */
            continue;
        }
        uint32_t taken = (uint32_t)sink->write(sink->ctx, queue + offset, n);
        tail += taken;
        if (taken < n) {
            return;  /* sink full: back to the ring */
        }
    }
}

STREAM_HOT bool tx_queue_empty(void)
{
    return head == tail;
}

STREAM_HOT uint32_t tx_queue_consumed(void)
{
    return tail;
}

STREAM_HOT uint32_t tx_queue_fill_permille(void)
{
    return (head - tail) * 1000u / TX_QUEUE_SIZE;
}
