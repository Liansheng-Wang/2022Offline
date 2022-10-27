#pragma once

#include <control/controller.h>

namespace Competition{
  void run()
  {
    Actuator controller;
    controller.arm();
    controller.setMode("OFFBOARD");
    controller.takeoff(5.0);
  }
};