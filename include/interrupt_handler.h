#pragma once

#include <functional>

#include "pico/types.h"

/**
 * Only one callback can be registered for handling all GPIO interrupts, so this namespace provides
 * a way to register different callbacks for different GPIO pins.
 * In addition, it also provides a way to register function objects, meaning that different data can
 * be accessed, based on the pin that triggered the interrupt.
*/
namespace interrupts {

void register_callback(uint gpio, uint32_t event_mask, const std::function<void(uint, uint32_t)>&);
void register_callback(uint gpio, uint32_t event_mask, const std::function<void(uint32_t)>&);
void register_callback(uint gpio, uint32_t event_mask, const std::function<void()>&);
void unregister_callback(uint gpio, uint32_t event_mask);

}  // namespace interrupts
