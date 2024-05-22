#pragma once

#include <functional>
#include <string>

namespace parameters {

/**
 * Registers a parameter that can be updated via serial communication.
 * @param name The name of the parameter.
 * @param setter A function that will be called with each new value of the parameter.
 */
void register_parameter(const std::string& name, const std::function<void(float)>& setter);

/** Removes a parameter from the list of parameters that can be updated. */
void unregister_parameter(const std::string& name);

/**
 * Starts polling the serial port for parameter updates.
 * This function will not return, so it should be executed on core1
 */
void start_updating();

}
