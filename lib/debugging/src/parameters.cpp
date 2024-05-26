#include "parameters.h"

#include <iostream>
#include <map>

#include "pico/stdlib.h"

static std::map<std::string, std::function<void(float)>> parameter_setters;

namespace parameters {

void register_parameter(const std::string& name, const std::function<void(float)>& setter) {
  parameter_setters.emplace(name, setter);
}

void unregister_parameter(const std::string& name) { parameter_setters.erase(name); }

void start_updating() {
  // list listened parameteters
  printf("Listening to parameters:\n");
  for (const auto& [name, _] : parameter_setters) printf("- %s\n", name.c_str());

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
