#pragma once;

#include <uav_control/controller.h>

namespace Competition{
  void run()
  {
    Controller controller;
    controller.arm();
    controller.setMode("OFFBOARD");
    controller.takeoff(5.0);
  }
};