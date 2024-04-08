#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

// GPIO pin numbers
const uint ENA_PIN = 2;
const uint IN1_PIN = 3;
const uint IN2_PIN = 4;


struct Slice {
    Slice(uint slice_num, uint frequency = 1000)
        : slice_num(slice_num)
    {
        set_frequency(frequency);
        pwm_set_enabled(slice_num, true);
    }

    ~Slice() {
        pwm_set_enabled(slice_num, false);
    }

    static Slice forPin(uint pin, uint frequency = 1000) {
        return Slice(pwm_gpio_to_slice_num(pin), frequency);
    }

    void set_frequency(uint frequency) {
        uint32_t f_sys = clock_get_hz(clk_sys); // typically 125'000'000 Hz
        uint32_t f_clock = 1000000UL;
        float divider = (float)f_sys / f_clock;
        pwm_set_clkdiv(slice_num, divider);
        wrap = f_clock / frequency;
        pwm_set_wrap(slice_num, wrap - 1); // -1 because counter starts at 0
    }

    uint slice_num;
    uint16_t wrap = 0xffff;
};

struct Motor {
    Motor(uint enable, uint in1, uint in2, Slice slice, uint duty = 0)
        : pin_enable(enable), pin_in1(in1), pin_in2(in2), slice(pwm_gpio_to_slice_num(pin_enable)), channel(pwm_gpio_to_channel(pin_enable))
    {
        gpio_init(pin_enable);
        gpio_init(pin_in1);
        gpio_init(pin_in2);
        gpio_set_function(pin_enable, GPIO_FUNC_PWM);
        gpio_set_dir(pin_in1, GPIO_OUT);
        gpio_set_dir(pin_in2, GPIO_OUT);
        set_duty(duty);
        stop();
    }

    void set_duty(uint duty) {
        uint16_t level = slice.wrap * duty / 100;
        pwm_set_chan_level(slice.slice_num, channel, level - 1); // -1 because counter starts at 0
    }

    void forward() {
        gpio_put(pin_in1, 1);
        gpio_put(pin_in2, 0);
    }

    void backward() {
        gpio_put(pin_in1, 0);
        gpio_put(pin_in2, 1);
    }

    void stop() {
        gpio_put(pin_in1, 0);
        gpio_put(pin_in2, 0);
    }

    uint pin_enable, pin_in1, pin_in2;
    Slice slice;
    uint channel;
};

int main() {
    stdio_init_all();

    auto slice = Slice::forPin(ENA_PIN);
    auto motor = Motor(ENA_PIN, IN1_PIN, IN2_PIN, slice);
    motor.set_duty(100);

    while (true) {
        sleep_ms(2000);
        motor.forward();
        sleep_ms(2000);
        motor.stop();
    }

    return 0;
}
