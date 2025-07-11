
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
int Edges::_sync = 0;
int Edges::_rise = 0;


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


// sync=0: waiting for something not a zero or one
//      0/1:    stay in sync=0
//      other:  go to sync=1
// sync=1: waiting for rise/fall (must be zero or one)
//      0/1:    note rise/fall, go to sync=2
//      other:  stay in sync=1
// sync=2: waiting for timestamp (might be zero or one)
//      any:    return timestamp and rise/fall, go to sync=1
bool Edges::get_tick(int& rise, uint32_t& tick)
{
    int32_t data;
    if (!queue_try_remove(&_queue, &data))
        return false;

    if (_sync == 0) {
        if (data != 0 && data != 1)
            _sync = 1;
        return false;

    } else if (_sync == 1) {
        if (data == 0 || data == 1) {
            _rise = data;
            _sync = 2;
        }
        return false;

    } else {
        xassert(_sync == 2);
        rise = _rise;
        // change from down counter to up counter
        tick = uint32_t(-data);
        _sync = 1;
        return true;

    }
}


uint32_t Edges::div()
{
    return edges_div;
}


void Edges::pio_irq_handler()
{
    // read available data and put them in the queue
    while (!pio_sm_is_rx_fifo_empty(_pio, _sm)) {
        uint32_t data = pio_sm_get_blocking(_pio, _sm);
        (void)queue_try_add(&_queue, &data);
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
