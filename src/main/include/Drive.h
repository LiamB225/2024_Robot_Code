// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Drive/MecanumDrive.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class Drive {
 public:
  Drive();

  void Cartesian(double drivePower, double strafePower, double turnPower);

  rev::CANSparkMax frontLeftMotor {OperatorConstants::frontLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRightMotor {OperatorConstants::frontRightID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backLeftMotor {OperatorConstants::backLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backRightMotor {OperatorConstants::backRightID, rev::CANSparkMax::MotorType::kBrushless};
  frc::MecanumDrive *myMecanumDrive;
};
