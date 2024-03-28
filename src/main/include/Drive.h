// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Drive/MecanumDrive.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/estimator/MecanumDrivePoseEstimator.h>
#include <rev/CANSparkMax.h>
#include <vector>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/math.h>
#include <units/voltage.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/Timer.h>
#include "Constants.h"

class Drive {
 public:
  Drive();

  frc::TrapezoidProfile<units::meters> *XProfile;
  frc::TrapezoidProfile<units::meters>::Constraints constraintsX{ 0.5_mps, 0.5_mps_sq };
  units::meter_t setpointX;

  frc::TrapezoidProfile<units::meters> *YProfile;
  frc::TrapezoidProfile<units::meters>::Constraints constraintsY{ 0.5_mps, 0.5_mps_sq };
  units::meter_t setpointY;

  frc::TrapezoidProfile<units::degrees> *RotProfile;
  frc::TrapezoidProfile<units::degrees>::Constraints constraintsRot{ 20_deg_per_s, 20_deg_per_s / 1_s };
  units::degree_t setpointRot;

  frc::Timer ProfileTimer;

  double kPFrontLeft = 0.20598;
  double kPFrontRight = 0.17521;
  double kPBackLeft = 0.36851;
  double kPBackRight = 0.15217;

  frc::PIDController frontLeftPID{ kPFrontLeft, 0.0, 0.0 };
  frc::SimpleMotorFeedforward<units::meters> frontLeftFF { 0.28424_V, 2.254_V * 1_s / 1_m, 0.34673_V * 1_s * 1_s / 1_m };
  units::meters_per_second_t frontLeftVelocity;

  frc::PIDController frontRightPID{ kPFrontRight, 0.0, 0.0 };
  frc::SimpleMotorFeedforward<units::meters> frontRightFF { 0.29889_V, 2.3081_V * 1_s / 1_m, 0.29396_V * 1_s * 1_s / 1_m };
  units::meters_per_second_t frontRightVelocity;

  frc::PIDController backLeftPID{ kPBackLeft, 0.0, 0.0 };
  frc::SimpleMotorFeedforward<units::meters> backLeftFF { 0.30486_V, 2.247_V * 1_s / 1_m, 0.35837_V * 1_s * 1_s / 1_m };
  units::meters_per_second_t backLeftVelocity;

  frc::PIDController backRightPID{ kPBackRight, 0.0, 0.0 };
  frc::SimpleMotorFeedforward<units::meters> backRightFF { 0.23311_V, 2.4556_V * 1_s / 1_m, 0.2923_V * 1_s * 1_s / 1_m };
  units::meters_per_second_t backRightVelocity;
  
  void Cartesian(double drivePower, double strafePower, double turnPower);
  void SetTarget(double targetXPos, double targetYPos, double targetRotPos);
  void Track(std::vector<double> currentPos);
  void SetVoltages();

  rev::CANSparkMax frontLeftMotor {OperatorConstants::frontLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder frontLeftEncoder = frontLeftMotor.GetEncoder();
  rev::CANSparkMax frontRightMotor {OperatorConstants::frontRightID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder frontRightEncoder = frontRightMotor.GetEncoder();
  rev::CANSparkMax backLeftMotor {OperatorConstants::backLeftID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder backLeftEncoder = backLeftMotor.GetEncoder();
  rev::CANSparkMax backRightMotor {OperatorConstants::backRightID, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder backRightEncoder = backRightMotor.GetEncoder();
  frc::MecanumDrive *myMecanumDrive;

  frc::Translation2d frontLeftLocation { 0.1778_m, 0.2873375_m };
  frc::Translation2d frontRightLocation { 0.1778_m, -0.2873375_m };
  frc::Translation2d backLeftLocation { -0.1778_m, 0.2873375_m };
  frc::Translation2d backRightLocation { -0.1778_m, -0.2873375_m };
  frc::MecanumDriveKinematics m_kinematics { frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation };
  frc::ADIS16470_IMU gyro;
  frc::MecanumDriveWheelPositions notWheelPositions { 0.0_m, 0.0_m, 0.0_m, 0.0_m};
    frc::Pose2d notPosition;
    frc::MecanumDrivePoseEstimator m_poseEstimator { m_kinematics, gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kYaw), notWheelPositions, notPosition};
};
