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
    if(runOnceFlywheels)
    {
        leftShooterMotor.Set(1.0);
        rightShooterMotor.Set(1.0);
        runOnceFlywheels = false;
    }
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
    runOnceFlywheels = true;
}

//Intake
void Shooter::IntakeIn()
{
    if(runOnceIntake)
    {
        topIntakeMotor.Set(1.0);
        bottomIntakeMotor.Set(1.0);
        runOnceIntake = false;
    }
}

void Shooter::IntakeOut()
{
    if(runOnceIntake)
    {
        topIntakeMotor.Set(-1.0);
        bottomIntakeMotor.Set(-1.0);
        runOnceIntake = false;
    }
}

void Shooter::StopIntake()
{
    timer.Reset();
    topIntakeMotor.StopMotor();
    bottomIntakeMotor.StopMotor();
    runOnceIntake = true;
}

bool Shooter::GetIntakeSensor()
{
    return intakeSensor.Get();
}

//Elevator Manual Control
void Shooter::AngleUp()
{
    if(runOnceElevator)
    {
        elevatorMotor.Set(1.0);
        runOnceElevator = false;
    }
}

void Shooter::AngleDown()
{
    if(runOnceElevator)
    {
        elevatorMotor.Set(-1.0);
        runOnceElevator = false;
    }
}

void Shooter::StopElevator()
{
    elevatorMotor.StopMotor();
    runOnceElevator = true;
}

bool Shooter::ZeroElevator()
{
    
    if(!elevatorLimit.Get())
    {
        elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        StopElevator();
        elevatorEncoder.SetPosition(0.0);
        runOnceZero = true;
        return true;
    }
    else
    {
        if(runOnceZero)
        {
            elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
            runOnceZero = false;
            elevatorMotor.Set(-1.0);
        }
        return false;
    }
}

void Shooter::GetPosition()
{
    frc::SmartDashboard::PutNumber("Encoder Value", elevatorEncoder.GetPosition());
}