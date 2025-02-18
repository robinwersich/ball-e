#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

int main() {
  stdio_init_all();
  if (cyw43_arch_init()) {
    printf("Wi-Fi init failed");
    return -1;
  }
  while (true) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("LED on!\n");
    sleep_ms(250);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    printf("LED off!\n");
    sleep_ms(250);
  }
}
