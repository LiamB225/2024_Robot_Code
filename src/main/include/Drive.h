// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Drive/MecanumDrive.h>
#include <rev/CANSparkMax.h>
#include <vector>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/math.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

class Drive {
 public:
  Drive();

  frc::TrapezoidProfile<units::meters>::Constraints constraintsX{ 1_mps, 1_mps_sq};
  frc::TrapezoidProfile<units::meters>::Constraints constraintsY{ 1_mps, 1_mps_sq};
  frc::TrapezoidProfile<units::degrees>::Constraints constraintsRot{ 20_deg_per_s, 10_deg_per_s / 1_s};

  frc::ProfiledPIDController<units::meters> pidX { 1.0, 0.0, 0.0, constraintsX };
  frc::ProfiledPIDController<units::meters> pidY { 1.0, 0.0, 0.0, constraintsY };
  frc::ProfiledPIDController<units::degrees> pidRot { 1.0, 0.0, 0.0, constraintsRot };

  void Cartesian(double drivePower, double strafePower, double turnPower);
  void SetTarget(double targetXPos, double targetYPos, double targetRotPos);
  void Track(std::vector<double> currentPos);
  void EndTargeting();

  rev::CANSparkMax frontLeftMotor {OperatorConstants::frontLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRightMotor {OperatorConstants::frontRightID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backLeftMotor {OperatorConstants::backLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backRightMotor {OperatorConstants::backRightID, rev::CANSparkMax::MotorType::kBrushless};
  frc::MecanumDrive *myMecanumDrive;
};
