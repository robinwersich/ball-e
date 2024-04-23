#include "parameters.h"

#include <iostream>
#include <map>

#include "pico/stdlib.h"
#include "pico/multicore.h"

namespace {

std::map<std::string, std::function<void(float)>> parameter_setters;

void update_parameters() {
  while (true) {
    std::string command;
    std::cin >> command;
    if (command != "set") {
      printf("Invalid command: %s\n", command.c_str());
      continue;
    }

    std::string name;
    float value;
    std::cin >> name >> value;

    if (parameter_setters.contains(name)) {
      parameter_setters[name](value);
    } else {
      printf("Unknown parameter: %s\n", name.c_str());
    }
  }
}

}

namespace parameters {

void register_parameter(const std::string& name, const std::function<void(float)>& setter) {
  parameter_setters.emplace(name, setter);
}

void unregister_parameter(const std::string& name) { parameter_setters.erase(name); }

void start_updating() {
  multicore_launch_core1(update_parameters);
}

}
