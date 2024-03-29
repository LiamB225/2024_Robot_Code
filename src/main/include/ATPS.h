// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>
#include <iostream>
#include <cmath>

class ATPS {
 public:
  ATPS();

  nt::NetworkTableInstance ntinst;

  std::vector<double> PositionSpeaker();
  double ElevatorHeight();
  double distance = 0.0;
  double redCenterX = 8.308467;
  double redCenterY = 1.442593;
};
