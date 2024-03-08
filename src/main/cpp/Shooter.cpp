// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter()
{
    topIntakeMotor.SetInverted(true);
    rightShooterMotor.SetInverted(true);
    elevatorMotor.SetInverted(true);
    elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 500.0);
    elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    elevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 0.0);
    timer.Start();
}

//Flywheels
void Shooter::Shoot()
{
    timer.Reset();
    leftShooterMotor.Set(1.0);
    rightShooterMotor.Set(1.0);
    frc::SmartDashboard::PutNumber("Left Velocity", leftShooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Right Velocity", rightShooterEncoder.GetVelocity());
    if(timer.Get() > 2_s)
    {
        IntakeIn();
    }
}

void Shooter::StopShooting()
{
    leftShooterMotor.StopMotor();
    rightShooterMotor.StopMotor();
}

//Intake
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

bool Shooter::GetIntakeSensor()
{
    return intakeSensor.Get();
}

//Elevator Manual Control
void Shooter::AngleUp()
{
    elevatorMotor.Set(1.0);
}

void Shooter::AngleDown()
{
    elevatorMotor.Set(-1.0);
}

void Shooter::StopElevator()
{
    elevatorMotor.StopMotor();
}

bool Shooter::ZeroElevator()
{
    
    if(!elevatorLimit.Get())
    {
        elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        StopElevator();
        elevatorEncoder.SetPosition(0.0);
        runonce = true;
        return true;
    }
    else
    {
        if(runonce)
        {
            elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
            runonce = false;
        }
        elevatorMotor.Set(-1.0);
        return false;
    }
}

void Shooter::GetPosition()
{
    frc::SmartDashboard::PutNumber("Encoder Value", elevatorEncoder.GetPosition());
}