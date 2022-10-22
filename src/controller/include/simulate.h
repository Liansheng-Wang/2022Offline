#pragma once

#include <uav_control/controller.h>

namespace Simulate{
  void run()
  {
    Controller controller;
    controller.arm();
    controller.setMode("OFFBOARD");
    controller.takeoff(2.0);

    





  }
};