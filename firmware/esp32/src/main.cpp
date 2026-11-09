#include "rb1/esp32/App.hpp"

namespace {
rb1::esp32::App firmware;
}

void setup() {
  (void)firmware.begin();
}

void loop() {
  firmware.loop();
}
