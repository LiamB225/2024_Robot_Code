// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <units/time.h>

#include "Constants.h"

class Shooter {
 public:
  Shooter();

  //Flywheels
  void Shoot(double elevatorHeight, double angleError, double actualAngle);
  void StopShooting(bool runWheels);
  // bool IsDone();
  bool runOnceFlywheels = true;
  bool runOnceIdle = true;
  rev::CANSparkMax leftShooterMotor {OperatorConstants::leftShooterID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder leftShooterEncoder = leftShooterMotor.GetEncoder();
  rev::CANSparkMax rightShooterMotor {OperatorConstants::rightShooterID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder rightShooterEncoder = rightShooterMotor.GetEncoder();
  rev::SparkPIDController leftPID = leftShooterMotor.GetPIDController();
  rev::SparkPIDController rightPID = rightShooterMotor.GetPIDController();
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

  //Elevator Control
  void AngleUp();
  void AngleDown();
  void StopElevator();
  void ZeroElevator();
  void GetPosition();
  void AutoElevatorControl(double elevatorHeight);
  bool runOnceElevator = true;
  bool runOnceZero = true;

  rev::CANSparkMax elevatorMotor {OperatorConstants::elevatorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder elevatorEncoder = elevatorMotor.GetEncoder();
  rev::SparkPIDController elevatorPID = elevatorMotor.GetPIDController();
  frc::DigitalInput elevatorLimit {OperatorConstants::limitSwitchID};
};
