
#include <Arduino.h>

#include "sys_led.h"
#include "xassert.h"

#include "edges.h"

// Connect some signal source to this gpio (a button or signal generator).
// Make it slow, because there's a printf for every change.
static const int gpio_in = 7;

static const uint tick_hz = 10'000'000;


void setup()
{
    Serial.begin(115200);

    SysLed::begin();
    SysLed::pattern(50, 950); // indicates "waiting for serial"

    while (!Serial)
        SysLed::loop();

    SysLed::off();

    // the conditions for these asserts might not be necessary
    // but this is the only config tested
    const uint32_t pio_clk_hz = clock_get_hz(clk_sys);
    xassert((pio_clk_hz % Edges::div()) == 0);
    const uint32_t pio_div_hz = pio_clk_hz / Edges::div();
    xassert((pio_div_hz % tick_hz) == 0);

    Edges::setup(gpio_in, tick_hz);

    if ((tick_hz % 1000000) == 0)
        Serial.printf("tick_hz=%uMHz\n", tick_hz / 1000000);
    else if ((tick_hz % 1000) == 0)
        Serial.printf("tick_hz=%uKHz\n", tick_hz / 1000);
    else
        Serial.printf("tick_hz=%u\n", tick_hz);
}


void loop()
{
    SysLed::loop();

    // wait for a new edge
    uint32_t tick;
    if (!Edges::get_tick(tick))
        return;

    // scale current time to be same units as tick
    // (assumes tick_hz >= 1'000'000)
    uint32_t now = micros() * (tick_hz / 1'000'000);

    static uint32_t now_last = now;

    static uint32_t tick_last = tick;

    // we always get one edge at startup; skip it
    static bool skip_first = true;
    if (skip_first) {
        skip_first = false;
        return;
    }

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

    now_last = now;
    tick_last = tick;

    // If this ever prints, it's probably rx fifo full and that's a problem.
    // (the interrupt handler should easily keep up)
    uint32_t fdebug = Edges::pio()->fdebug;
    if (fdebug != 0) {
        Serial.printf(" fdebug=0x%08x", fdebug);
        Edges::pio()->fdebug = fdebug; // clear everything
    }

    Serial.printf("\n");
}
