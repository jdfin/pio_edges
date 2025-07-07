#pragma once

#include <Arduino.h>
#include "hardware/pio.h"
#include "pico/util/queue.h"


class Edges {

    public:

        static void setup(int gpio, uint tick_hz);

        static inline bool get_tick(uint32_t& tick)
        {
            if (!queue_try_remove(&_queue, &tick))
                return false;
            // change from 32-bit down counter to 32-bit up counter
            tick = -tick;
            return true;
        }

        // below here is mostly for debugging

        static uint32_t div();

        static inline PIO pio() { return _pio; }
        static inline uint sm() { return _sm; }

    private:

        static PIO _pio; // pointer to the hardware registers

        static uint _sm;

        static const uint queue_len = 64;

        static queue_t _queue;

        static void pio_irq_handler();

        static void pio_irq_init();

}; // class Edges
