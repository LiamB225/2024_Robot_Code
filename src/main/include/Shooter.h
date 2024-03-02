// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>

#include "Constants.h"

class Shooter {
 public:
  Shooter();

  void Shoot();
  void StopShooting();
  void IntakeIn();
  void IntakeOut();
  void StopIntake();
  void GetIntakeSensor();

  rev::CANSparkMax leftShooterMotor {OperatorConstants::leftShooterID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder leftShooterEncoder = leftShooterMotor.GetEncoder();
  rev::CANSparkMax rightShooterMotor {OperatorConstants::rightShooterID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder rightShooterEncoder = rightShooterMotor.GetEncoder();
  rev::CANSparkMax topIntakeMotor {OperatorConstants::topIntakeID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax bottomIntakeMotor {OperatorConstants::bottomIntakeID, rev::CANSparkMax::MotorType::kBrushed};
  frc::DigitalInput intakeSensor {OperatorConstants::intakeSensorID};
};
