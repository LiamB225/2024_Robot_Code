// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drive.h"

Drive::Drive()
{
    myMecanumDrive = new frc::MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    frontRightMotor.SetInverted(true);
    backRightMotor.SetInverted(true);
    frontLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    frontRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    backLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    backRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    
    frontLeftEncoder.SetVelocityConversionFactor(0.00797964534 / 9.13);
    frontRightEncoder.SetVelocityConversionFactor(0.00797964534 / 9.13);
    backLeftEncoder.SetVelocityConversionFactor(0.00797964534 / 9.13);
    backRightEncoder.SetVelocityConversionFactor(0.00797964534 / 9.13);

    frontLeftEncoder.SetPositionConversionFactor(0.4787787204 / 9.13);
    frontRightEncoder.SetPositionConversionFactor(0.4787787204 / 9.13);
    backLeftEncoder.SetPositionConversionFactor(0.4787787204 / 9.13);
    backRightEncoder.SetPositionConversionFactor(0.4787787204 / 9.13);

    frontLeftPID.SetPID(kPFrontLeft, 0.0, 0.0);
    frontRightPID.SetPID(kPFrontRight, 0.0, 0.0);
    backLeftPID.SetPID(kPBackLeft, 0.0, 0.0);
    backRightPID.SetPID(kPBackRight, 0.0, 0.0);

    XProfile = new frc::TrapezoidProfile<units::meters>(constraintsX);
    YProfile = new frc::TrapezoidProfile<units::meters>(constraintsY);
    RotProfile = new frc::TrapezoidProfile<units::degrees>(constraintsRot);
    ProfileTimer.Start();
}

void Drive::Cartesian(double drivePower, double strafePower, double turnPower)
{
    myMecanumDrive->DriveCartesian(drivePower, strafePower, turnPower);
}

void Drive::SetTarget(double targetXPos, double targetYPos, double targetRotPos)
{
    ProfileTimer.Reset();
    setpointX = (units::meter_t)(targetXPos);
    setpointY = (units::meter_t)(targetYPos);
    setpointRot = (units::degree_t)(targetRotPos);
}

void Drive::ResetPosition(std::vector<double> currentPos)
{
    units::meter_t currentflpos = (units::meter_t)(frontLeftEncoder.GetPosition());
    units::meter_t currentfrpos = (units::meter_t)(frontRightEncoder.GetPosition());
    units::meter_t currentblpos = (units::meter_t)(backLeftEncoder.GetPosition());
    units::meter_t currentbrpos = (units::meter_t)(backRightEncoder.GetPosition());
    frc::MecanumDriveWheelPositions wheelPositions { currentflpos, currentfrpos, currentblpos, currentbrpos };
    frc::Rotation2d m_rotation { (units::degree_t)(currentPos[2]) };
    frc::Pose2d m_visionPosition { (units::meter_t)(currentPos[0]), (units::meter_t)(currentPos[1]), m_rotation };
    m_poseEstimator.ResetPosition(gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kYaw), wheelPositions, m_visionPosition);
}

void Drive::EstimatePosition(std::vector<double> currentPos)
{
    units::meter_t currentflpos = (units::meter_t)(frontLeftEncoder.GetPosition());
    units::meter_t currentfrpos = (units::meter_t)(frontRightEncoder.GetPosition());
    units::meter_t currentblpos = (units::meter_t)(backLeftEncoder.GetPosition());
    units::meter_t currentbrpos = (units::meter_t)(backRightEncoder.GetPosition());
    frc::MecanumDriveWheelPositions wheelPositions { currentflpos, currentfrpos, currentblpos, currentbrpos };

    frc::Rotation2d m_rotation { (units::degree_t)(currentPos[2]) };
    frc::Pose2d m_visionPosition { (units::meter_t)(currentPos[0]), (units::meter_t)(currentPos[1]), m_rotation };

    if(currentPos[0] != 0.0 || currentPos[1] != 0.0 || currentPos[2] != 0.0)
    {
        m_poseEstimator.AddVisionMeasurement(m_visionPosition, ProfileTimer.Get());
    }

    m_poseEstimator.UpdateWithTime(ProfileTimer.Get(), gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kYaw), wheelPositions);
}

void Drive::Track()
{
    units::meters_per_second_t currentflvel = (units::meters_per_second_t)(frontLeftEncoder.GetVelocity());
    units::meters_per_second_t currentfrvel = (units::meters_per_second_t)(frontRightEncoder.GetVelocity());
    units::meters_per_second_t currentblvel = (units::meters_per_second_t)(backLeftEncoder.GetVelocity());
    units::meters_per_second_t currentbrvel = (units::meters_per_second_t)(backRightEncoder.GetVelocity());
    frc::MecanumDriveWheelSpeeds wheelSpeeds { currentflvel, currentfrvel, currentblvel, currentbrvel };
    auto [forward, sideways, angular] = m_kinematics.ToChassisSpeeds(wheelSpeeds);

    frc::Pose2d robotPos = m_poseEstimator.GetEstimatedPosition();

    frc::SmartDashboard::PutNumber("estimated X", robotPos.X().value());
    frc::SmartDashboard::PutNumber("estimated Y", robotPos.Y().value());
    frc::SmartDashboard::PutNumber("estimated Rot", robotPos.Rotation().Degrees().value());

    currentTime = ProfileTimer.Get();

    auto velocityX = XProfile->Calculate(
       currentTime - lastTime,
       frc::TrapezoidProfile<units::meters>::State { robotPos.X(), forward},
       frc::TrapezoidProfile<units::meters>::State { setpointX, 0.0_mps}
    );

    auto velocityY = YProfile->Calculate(
       currentTime - lastTime,
       frc::TrapezoidProfile<units::meters>::State { robotPos.Y(), sideways},
       frc::TrapezoidProfile<units::meters>::State { setpointY, 0.0_mps}
    );

    // auto velocityRot = RotProfile->Calculate(
    //    currentTime - lastTime,
    //    frc::TrapezoidProfile<units::degrees>::State { robotPos.Rotation().Degrees(), (units::degrees_per_second_t)(angular * 57.2958)},
    //    frc::TrapezoidProfile<units::degrees>::State { setpointRot, 0.0_deg_per_s}
    // );

    double newX = velocityX.velocity.value() * cos(robotPos.Rotation().Radians().value()) + velocityY.velocity.value() * sin(robotPos.Rotation().Radians().value());
    double newY = velocityX.velocity.value() * sin(robotPos.Rotation().Radians().value()) - velocityY.velocity.value() * cos(robotPos.Rotation().Radians().value());

    frc::SmartDashboard::PutNumber("Xvelocity", velocityX.velocity.value());
    frc::SmartDashboard::PutNumber("newX", newX);
    frc::SmartDashboard::PutNumber("newY", newY);

    frc::ChassisSpeeds chassisSpeeds {(units::meters_per_second_t)(newX), (units::meters_per_second_t)(0.0), (units::degrees_per_second_t)(0.0)};
    auto [fl, fr, bl, br] = m_kinematics.ToWheelSpeeds(chassisSpeeds);
    frontLeftVelocity = fl;
    frontRightVelocity = fr;
    backLeftVelocity = bl;
    backRightVelocity = br;

    lastTime = currentTime;
}

void Drive::SetVoltages()
{
    frontLeftMotor.SetVoltage(frontLeftFF.Calculate(frontLeftVelocity) + (units::volt_t)(frontLeftPID.Calculate(frontLeftEncoder.GetVelocity(), frontLeftVelocity.value())));
    frontRightMotor.SetVoltage(frontRightFF.Calculate(frontRightVelocity) + (units::volt_t)(frontRightPID.Calculate(frontRightEncoder.GetVelocity(), frontRightVelocity.value())));
    backLeftMotor.SetVoltage(backLeftFF.Calculate(backLeftVelocity) + (units::volt_t)(backLeftPID.Calculate(backLeftEncoder.GetVelocity(), backLeftVelocity.value())));
    backRightMotor.SetVoltage(backRightFF.Calculate(backRightVelocity) + (units::volt_t)(backRightPID.Calculate(backRightEncoder.GetVelocity(), backRightVelocity.value())));
    frc::SmartDashboard::PutNumber("FrontleftActual", frontLeftEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("FrontrightActual", frontRightEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("backleftActual", backLeftEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("backrightActual", backRightEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("FrontleftExpected", frontLeftVelocity.value());
    frc::SmartDashboard::PutNumber("FrontrightExpected", frontRightVelocity.value());
    frc::SmartDashboard::PutNumber("backleftExpected", backLeftVelocity.value());
    frc::SmartDashboard::PutNumber("backrightExpected", backRightVelocity.value());
}