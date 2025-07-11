
#include <Arduino.h>
#include "hardware/pwm.h"

#include "sys_led.h"
#include "xassert.h"

#include "edges.h"

// Connect gpio_out and gpio_in. gpio_out will output a pulse train at 20 Hz:
// 10 msec high, 40 msec low. The edge messages from gpio_in should show that
// exactly since they run from the same clock.

static const int gpio_out = 6;
static const int gpio_in = 7;

static const uint pio_tick_hz = 50'000'000;
static const uint32_t pio_tick_ns = 20;


static void setup_pwm()
{
    gpio_set_function(gpio_out, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(gpio_out);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, 200);
    pwm_config_set_wrap(&config, 49999); // period = 50 msec (20 Hz)
    pwm_init(slice, &config, false);

    uint channel = pwm_gpio_to_channel(gpio_out);

    pwm_set_chan_level(slice, channel, 10000); // 10 msec high, 40 msec low

    pwm_set_enabled(slice, true);
}


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
    xassert((pio_div_hz % pio_tick_hz) == 0);

    Edges::setup(gpio_in, pio_tick_hz);

    if ((pio_tick_hz % 1000000) == 0)
        Serial.printf("pio_tick_hz=%uMHz\n", pio_tick_hz / 1000000);
    else if ((pio_tick_hz % 1000) == 0)
        Serial.printf("pio_tick_hz=%uKHz\n", pio_tick_hz / 1000);
    else
        Serial.printf("pio_tick_hz=%u\n", pio_tick_hz);

    setup_pwm();
}


void loop()
{
    SysLed::loop();

    // wait for a new edge
    int rise;
    uint32_t tick;
    if (!Edges::get_tick(rise, tick))
        return;

    static uint32_t tick_last = tick;

    uint32_t interval_ns = (tick - tick_last) * pio_tick_ns;

    // print using nsec, usec, or msec, whichever fits best
    Serial.printf("%s for ", rise == 1 ? "lo" : "hi");
    if ((interval_ns % 1'000'000) == 0)
        Serial.printf("%lu msec\n", interval_ns / 1'000'000);
    else if ((interval_ns % 1'000) == 0)
        Serial.printf("%lu usec\n", interval_ns / 1'000);
    else
        Serial.printf("%lu nsec\n", interval_ns);

    tick_last = tick;
}
