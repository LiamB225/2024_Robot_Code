// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <units/time.h>

#include "Constants.h"

class Shooter {
 public:
  Shooter();

  //Flywheels
  void Shoot();
  void StopShooting();
  bool runOnceFlywheels = true;
  rev::CANSparkMax leftShooterMotor {OperatorConstants::leftShooterID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder leftShooterEncoder = leftShooterMotor.GetEncoder();
  rev::CANSparkMax rightShooterMotor {OperatorConstants::rightShooterID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder rightShooterEncoder = rightShooterMotor.GetEncoder();
  frc::Timer timer;

  //Intake
  void IntakeIn();
  void IntakeOut();
  void StopIntake();
  bool GetIntakeSensor();
  bool runOnceIntake = true;
  rev::CANSparkMax topIntakeMotor {OperatorConstants::topIntakeID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax bottomIntakeMotor {OperatorConstants::bottomIntakeID, rev::CANSparkMax::MotorType::kBrushed};
  frc::DigitalInput intakeSensor {OperatorConstants::intakeSensorID};

  //Elevator Manual Control
  void AngleUp();
  void AngleDown();
  void StopElevator();
  bool ZeroElevator();
  void GetPosition();
  bool runOnceElevator = true;
  bool runOnceZero = true;

  rev::CANSparkMax elevatorMotor {OperatorConstants::elevatorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder elevatorEncoder = elevatorMotor.GetEncoder();
  frc::DigitalInput elevatorLimit {OperatorConstants::limitSwitchID};
};
