#define USE_INTERRUPTS

#include <Arduino.h>
#include "hardware/pio.h"

#ifdef USE_INTERRUPTS
#include "pico/util/queue.h"
#endif

#include "sys_led.h"
#include "xassert.h"

#include "edges.pio.h"

// Connect some signal source to this gpio (a button or signal generator).
// Make it slow, because there's a printf for every change.
static const int gpio_in = 16;

static const uint tick_hz = 10'000'000;

static PIO pio = NULL; // pointer to the hardware registers
static uint sm = UINT_MAX;

#ifdef USE_INTERRUPTS

static volatile uint irq_cnt = 0;

static queue_t queue;
static const int queue_size = 64;

static volatile uint queue_overrun = 0;


static void pio_irq_handler(void)
{
    irq_cnt++;

    // read available ticks and put them in the queue
    while(!pio_sm_is_rx_fifo_empty(pio, sm)) {
        uint32_t tick = pio_sm_get_blocking(pio, sm);
        if (!queue_try_add(&queue, &tick))
            queue_overrun++;
    }
}


static void pio_irq_init()
{
    xassert(pio != NULL);
    xassert(sm < NUM_PIO_STATE_MACHINES);

    // find free irq for PIOx_IRQ_0
    int8_t pio_irq = pio_get_irq_num(pio, 0);

    if (irq_get_exclusive_handler(pio_irq) != NULL) {
        // PIOx_IRQ_0 is in use exclusively, check PIOx_IRQ_1

        pio_irq = pio_get_irq_num(pio, 1);

        // fail if PIOx_IRQ_1 is also in use exclusively
        bool status = (irq_get_exclusive_handler(pio_irq) != NULL);
        xassert(status);
    }

    queue_init(&queue, sizeof(uint32_t), queue_size);

    irq_add_shared_handler(pio_irq, pio_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    // enable interrupt in this core's interrupt controller

    irq_set_enabled(pio_irq, true);

    // enable interrupt in the PIO

    const uint irq_idx = pio_irq - pio_get_irq_num(pio, 0);

    const pio_interrupt_source pis = pio_get_rx_fifo_not_empty_interrupt_source(sm);

    pio_set_irqn_source_enabled(pio, irq_idx, pis, true);
}

#endif // USE_INTERRUPTS


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();

    // these might not be necessary but this is the only config tested
    const uint32_t pio_clk_hz = clock_get_hz(clk_sys);
    xassert((pio_clk_hz % edges_div) == 0);
    const uint32_t pio_div_hz = pio_clk_hz / edges_div;
    xassert((pio_div_hz % tick_hz) == 0);

    uint offset;
    bool status =
        pio_claim_free_sm_and_add_program_for_gpio_range(&edges_program, &pio,
                                                         &sm, &offset,
                                                         gpio_in, 1, true);
    xassert(status);

    Serial.printf("pio=%p sm=%u offset=%u gpio_in=%d ", pio, sm, offset, gpio_in);

    if ((tick_hz % 1000000) == 0)
        Serial.printf("tick_hz=%uMHz", tick_hz / 1000000);
    else if ((tick_hz % 1000) == 0)
        Serial.printf("tick_hz=%uKHz", tick_hz / 1000);
    else
        Serial.printf("tick_hz=%u", tick_hz);

    Serial.printf("\n");

#ifdef USE_INTERRUPTS
    pio_irq_init();
#endif

    edges_program_init(pio, sm, offset, gpio_in, tick_hz);

    // this should be enabled by default; check it
    if (!gpio_is_input_hysteresis_enabled(gpio_in))
        Serial.printf("Schmitt trigger on GPIO %d DISABLED\n", gpio_in);
}


void loop()
{
    SysLed::loop();

    // wait for a tick from the interrupt handler
#ifdef USE_INTERRUPTS
    if (queue_is_empty(&queue))
        return;
#else
    if (pio_sm_is_rx_fifo_empty(pio, sm))
        return;
#endif

    // scale current time to be same units as tick
    uint32_t now = micros() * (tick_hz / 1'000'000);

    static uint32_t now_last = now;

    uint32_t tick;

#ifdef USE_INTERRUPTS
    (void)queue_try_remove(&queue, &tick);
#else
    tick = pio_sm_get_blocking(pio, sm);
#endif

    static uint32_t tick_last = tick;

    // we always get one edge at startup; skip it
    static bool skip_first = true;
    if (skip_first) {
        skip_first = false;
        return;
    }

    // change from 32-bit down counter to 32-bit up counter
    uint64_t tick_up = uint64_t(UINT32_MAX) + 1 - tick;
    tick = uint32_t(tick_up);

    // difference between CPU's tick and PIO's tick
    uint32_t delta = now - tick;

    // save first difference
    static uint32_t delta_1 = delta;

    // difference between this difference and first difference
    int32_t d_delta = int32_t(delta - delta_1);

    // Ignoring outliers, d_delta should stay in a tight range forever.
    //
    // Outliers happen (probably) because of other interrupt handlers. That's
    // okay; the PIO's tick is the correct one, and is why just interrupting
    // with GPIO edges and having the CPU create timestamp is less reliable.
    //
    // If the PIO ticks are 1 usec, d_delta should always be within a range of
    // a tick or two. If the PIO ticks are 0.1 usec, d_delta should always be
    // within a range of about 10. (Except for the outliers.)

    Serial.printf("now=%lu (+%lu) tick=%lu (+%lu) d_delta=%-3ld",
                  now, now - now_last, tick, tick - tick_last, d_delta);

#ifdef USE_INTERRUPTS
    Serial.printf(" irq_cnt=%u", irq_cnt);
#endif

    now_last = now;
    tick_last = tick;

    // If this ever prints, it's probably rx fifo full and that's a problem.
    uint32_t fdebug = pio->fdebug;
    if (fdebug != 0) {
        Serial.printf(" fdebug=0x%08x", fdebug);
        pio->fdebug = fdebug; // clear everything
    }

    Serial.printf("\n");
}
