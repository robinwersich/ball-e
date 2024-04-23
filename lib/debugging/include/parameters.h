#pragma once

#include <functional>

namespace parameters {

/**
 * Registers a parameter that can be updated via serial communication.
 * @param name The name of the parameter.
 * @param setter A function that will be called with each new value of the parameter.
 */
void register_parameter(const char* name, const std::function<void(float)>& setter);

/** Removes a parameter from the list of parameters that can be updated. */
void unregister_parameter(const char* name);

/**
 * Starts polling the serial port for parameter updates.
 * This will use the second core of the pico.
 */
void start_updating();

}
