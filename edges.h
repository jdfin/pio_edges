#pragma once

#include <Arduino.h>
#include "hardware/pio.h"
#include "pico/util/queue.h"


class Edges {

    public:

        static void setup(int gpio, uint tick_hz);

        static bool get_tick(int& rise, uint32_t& tick);

        // below here is mostly for debugging

        static uint32_t div();

        static inline PIO pio() { return _pio; }
        static inline uint sm() { return _sm; }

    private:

        static PIO _pio; // pointer to the hardware registers

        static uint _sm;

        static const uint queue_len = 64;

        static queue_t _queue;

        // We're in sync when we see something that is not a zero or one,
        // followed by a zero or one. The first tick will be the next thing
        // in the fifo.
        static int _sync;
        static int _rise;

        static void pio_irq_handler();

        static void pio_irq_init();

}; // class Edges
