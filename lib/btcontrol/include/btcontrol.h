#pragma once

#include <functional>

#include "uni_hid_device.h"

namespace btcontrol {

/**
 * Initializes bluetooth module and gamepad library.
 * @returns 0 on success, error code on failure.
 */
int init();

/** Runs the control loop. Will not return. */
void run_loop();

/** Registers a callback to execute when new gamepad data is available. */
void register_gampad_behavior(std::function<void(const uni_gamepad_t&)> on_gamepad_data);
/** Registers a callback to execute when new keyboard data is available. */
void register_keyboard_behavior(std::function<void(const uni_keyboard_t&)> on_keyboard_data);

}