
#include <Arduino.h>
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/util/queue.h"

#include "xassert.h"

#include "edges.pio.h"

#include "edges.h"

PIO Edges::_pio = NULL; // pointer to the hardware registers
uint Edges::_sm = UINT_MAX;
queue_t Edges::_queue;


void Edges::setup(int gpio, uint tick_hz)
{
    uint offset;

    bool status =
        pio_claim_free_sm_and_add_program_for_gpio_range(&edges_program, &_pio,
                                                         &_sm, &offset, gpio,
                                                         1, true);
    xassert(status);

    pio_irq_init();

    edges_program_init(_pio, _sm, offset, gpio, tick_hz);
}


uint32_t Edges::div()
{
    return edges_div;
}


void Edges::pio_irq_handler()
{
    // read available ticks and put them in the queue
    while (!pio_sm_is_rx_fifo_empty(_pio, _sm)) {
        uint32_t tick = pio_sm_get_blocking(_pio, _sm);
        (void)queue_try_add(&_queue, &tick);
    }
}


void Edges::pio_irq_init()
{
    xassert(_pio != NULL);
    xassert(_sm < NUM_PIO_STATE_MACHINES);

    // find free irq for PIOx_IRQ_0
    int8_t pio_irq = pio_get_irq_num(_pio, 0);

    if (irq_get_exclusive_handler(pio_irq) != NULL) {
        // PIOx_IRQ_0 is in use exclusively, check PIOx_IRQ_1

        pio_irq = pio_get_irq_num(_pio, 1);

        // fail if PIOx_IRQ_1 is also in use exclusively
        bool status = (irq_get_exclusive_handler(pio_irq) != NULL);
        xassert(status);
    }

    queue_init(&_queue, sizeof(uint32_t), queue_len);

    irq_add_shared_handler(pio_irq, pio_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    // enable interrupt in this core's interrupt controller

    irq_set_enabled(pio_irq, true);

    // enable interrupt in the PIO

    const uint irq_idx = pio_irq - pio_get_irq_num(_pio, 0);

    const pio_interrupt_source pis = pio_get_rx_fifo_not_empty_interrupt_source(_sm);

    pio_set_irqn_source_enabled(_pio, irq_idx, pis, true);
}
