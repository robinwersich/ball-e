#include "interrupt_handler.h"

#include <vector>

#include "pico/stdlib.h"

namespace {

struct CallbackEntry {
  uint gpio;
  uint32_t event_mask;
  std::function<void(uint, uint32_t)> callback;
};

static std::vector<CallbackEntry> callback_entries;

void handle_interrupt(uint gpio, uint32_t event_mask) {
  for (const auto& entry : callback_entries) {
    if (entry.gpio == gpio && (entry.event_mask & event_mask)) { entry.callback(gpio, event_mask); }
  }
}

}  // namespace

namespace interrupts {

void register_callback(
  uint gpio, uint32_t event_mask, const std::function<void(uint, uint32_t)>& callback
) {
  gpio_set_irq_enabled_with_callback(gpio, event_mask, true, handle_interrupt);
  callback_entries.push_back({gpio, event_mask, callback});
}

void register_callback(
  uint gpio, uint32_t event_mask, const std::function<void(uint32_t)>& callback
) {
  register_callback(gpio, event_mask, [callback](uint gpio, uint32_t event_mask) {
    callback(event_mask);
  });
}

void register_callback(uint gpio, uint32_t event_mask, const std::function<void()>& callback) {
  register_callback(gpio, event_mask, [callback](uint gpio, uint32_t event_mask) { callback(); });
}

void unregister_callback(uint gpio, uint32_t event_mask) {
  gpio_set_irq_enabled(gpio, event_mask, false);
  callback_entries.erase(
    std::remove_if(
      callback_entries.begin(), callback_entries.end(),
      [gpio, event_mask](const CallbackEntry& entry) {
        return entry.gpio == gpio && entry.event_mask == event_mask;
      }
    ),
    callback_entries.end()
  );
}

}  // namespace interrupts
