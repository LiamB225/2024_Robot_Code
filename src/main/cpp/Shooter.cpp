// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter()
{
    topIntakeMotor.SetInverted(true);
    rightShooterMotor.SetInverted(true);
}

void Shooter::Shoot()
{
    leftShooterMotor.Set(1.0);
    rightShooterMotor.Set(1.0);
    frc::SmartDashboard::PutNumber("Left Velocity", leftShooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Right Velocity", rightShooterEncoder.GetVelocity());
}

void Shooter::StopShooting()
{
    leftShooterMotor.StopMotor();
    rightShooterMotor.StopMotor();
}

void Shooter::IntakeIn()
{
    topIntakeMotor.Set(1.0);
    bottomIntakeMotor.Set(1.0);
}

void Shooter::IntakeOut()
{
    topIntakeMotor.Set(-1.0);
    bottomIntakeMotor.Set(-1.0);
}

void Shooter::StopIntake()
{
    topIntakeMotor.StopMotor();
    bottomIntakeMotor.StopMotor();
}