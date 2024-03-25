// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drive.h"

Drive::Drive()
{
    myMecanumDrive = new frc::MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    frontRightMotor.SetInverted(true);
    backRightMotor.SetInverted(true);
    
    frontLeftEncoder.SetVelocityConversionFactor(0.00797964534);
    frontRightEncoder.SetVelocityConversionFactor(0.00797964534);
    backLeftEncoder.SetVelocityConversionFactor(0.00797964534);
    backRightEncoder.SetVelocityConversionFactor(0.00797964534);

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

void Drive::Track(std::vector<double> currentPos)
{
    auto velocityX = XProfile->Calculate(
        ProfileTimer.Get(),
        frc::TrapezoidProfile<units::meters>::State { (units::meter_t)(currentPos[0]), 0.0_mps},
        frc::TrapezoidProfile<units::meters>::State { setpointX, 0.0_mps}
    );

    auto velocityY = YProfile->Calculate(
        ProfileTimer.Get(),
        frc::TrapezoidProfile<units::meters>::State { (units::meter_t)(currentPos[1]), 0.0_mps},
        frc::TrapezoidProfile<units::meters>::State { setpointY, 0.0_mps}
    );

    auto velocityRot = RotProfile->Calculate(
        ProfileTimer.Get(),
        frc::TrapezoidProfile<units::degrees>::State { (units::meter_t)(currentPos[2]), 0.0_deg_per_s},
        frc::TrapezoidProfile<units::degrees>::State { setpointRot, 0.0_deg_per_s}
    );

    m_speeds.Discretize(velocityX.velocity, velocityY.velocity, velocityRot.velocity * 0.0174533, 0.02_s);
    auto [fl, fr, bl, br] = m_kinematics.ToWheelSpeeds(m_speeds);
    frontLeftVelocity = fl;
    frontRightVelocity = fr;
    backLeftVelocity = bl;
    backRightVelocity = br;
}

void Drive::SetVoltages()
{
    frontLeftMotor.SetVoltage(frontLeftFF.Calculate(frontLeftVelocity) + (units::volt_t)(frontLeftPID.Calculate(frontLeftEncoder.GetVelocity(), frontLeftVelocity.value())));
    frontRightMotor.SetVoltage(frontRightFF.Calculate(frontRightVelocity) + (units::volt_t)(frontRightPID.Calculate(frontRightEncoder.GetVelocity(), frontRightVelocity.value())));
    backLeftMotor.SetVoltage(backLeftFF.Calculate(backLeftVelocity) + (units::volt_t)(backLeftPID.Calculate(backLeftEncoder.GetVelocity(), backLeftVelocity.value())));
    backRightMotor.SetVoltage(backRightFF.Calculate(backRightVelocity) + (units::volt_t)(backRightPID.Calculate(backRightEncoder.GetVelocity(), backRightVelocity.value())));
}