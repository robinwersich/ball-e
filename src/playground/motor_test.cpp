#include "pico/stdlib.h"
#include "motor.h"


// GPIO pin numbers
const uint ENA_PIN = 2;
const uint IN1_PIN = 3;
const uint IN2_PIN = 4;


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
