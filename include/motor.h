#include "pico/types.h"

struct Slice {
    Slice(uint slice_num, uint frequency = 1000);
    ~Slice();
    static Slice forPin(uint pin, uint frequency = 1000);

    void set_frequency(uint frequency);

    uint slice_num;
    uint16_t wrap = 0xffff;
};

struct Motor {
    Motor(uint enable, uint in1, uint in2, Slice slice, uint duty = 0);
    
    void set_duty(uint duty);
    void forward();
    void backward();
    void stop();

    uint pin_enable, pin_in1, pin_in2;
    Slice slice;
    uint channel;
};