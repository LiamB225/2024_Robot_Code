// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <vector>
#include <iostream>

class ATPS {
 public:
  ATPS();

  nt::NetworkTableInstance ntinst;

  std::vector<double> PositionSpeaker();
};
