#include "btcontrol.h"

#include "btstack_run_loop.h"
#include "pico/cyw43_arch.h"
#include "uni.h"

static std::function<void(const uni_gamepad_t&)> on_gamepad_data;
static std::function<void(const uni_keyboard_t&)> on_keyboard_data;

static void platform_init(int argc, const char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  logi("btcontrol: initializing...\n");
}

static void platform_on_init_complete(void) {
  logi("btcontrol: initializing complete\n");

  // Safe to call "unsafe" functions since they are called from BT thread
  uni_bt_enable_new_connections_unsafe(true);
  uni_bt_del_keys_unsafe();
  uni_property_dump_all();
}

static void platform_on_device_connected(uni_hid_device_t* d) {
  logi("btcontrol: device connected: %p\n", d);
  // Turn on LED to mark connection
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
}

static void platform_on_device_disconnected(uni_hid_device_t* d) {
  logi("btcontrol: device disconnected: %p\n", d);
  // Turn off LED to mark connection loss
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
}

static uni_error_t platform_on_device_ready(uni_hid_device_t* d) {
  logi("btcontrol: device ready: %p\n", d);
  return UNI_ERROR_SUCCESS;
}

static void platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
  switch (ctl->klass) {
    case UNI_CONTROLLER_CLASS_GAMEPAD:
      if (on_gamepad_data) on_gamepad_data(ctl->gamepad);
      else logi("Gamepad data received, but no callback registered\n");
      break;
    case UNI_CONTROLLER_CLASS_KEYBOARD:
      if (on_keyboard_data) on_keyboard_data(ctl->keyboard);
      else logi("Keyboard data received, but no callback registered\n");
      break;
    default: loge("Unsupported controller class: %d\n", ctl->klass); break;
  }
}

static const uni_property_t* platform_get_property(uni_property_idx_t idx) {
  ARG_UNUSED(idx);
  return NULL;
}

static void platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
  switch (event) {
    case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
      // When the "bt scanning" is on / off. Could be triggered by different events
      // Useful to notify the user
      logi("platform_on_oob_event: Bluetooth enabled: %d\n", (bool)(data));
      break;

    default: logi("platform_on_oob_event: unsupported event: 0x%04x\n", event);
  }
}

static struct uni_platform platform = {
  .name = "btcontrol",
  .init = platform_init,
  .on_init_complete = platform_on_init_complete,
  .on_device_connected = platform_on_device_connected,
  .on_device_disconnected = platform_on_device_disconnected,
  .on_device_ready = platform_on_device_ready,
  .on_controller_data = platform_on_controller_data,
  .get_property = platform_get_property,
  .on_oob_event = platform_on_oob_event,
};

namespace btcontrol {

int init() {
  if (auto error = cyw43_arch_init()) {
    loge("failed to initialise cyw43_arch\n");
    return error;
  }

  uni_platform_set_custom(&platform);
  return uni_init(0, nullptr);
}

void run_loop() { btstack_run_loop_execute(); }

void register_gampad_behavior(std::function<void(const uni_gamepad_t&)> on_gamepad_data) {
  ::on_gamepad_data = std::move(on_gamepad_data);
}

void register_keyboard_behavior(std::function<void(const uni_keyboard_t&)> on_keyboard_data) {
  ::on_keyboard_data = std::move(on_keyboard_data);
}

}
