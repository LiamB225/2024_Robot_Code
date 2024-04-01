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

    elevatorPID.SetP(0.5);
    elevatorPID.SetI(0.0);
    elevatorPID.SetD(0.0);
    elevatorPID.SetIZone(0.0);
    elevatorPID.SetFF(0.0);
    elevatorPID.SetOutputRange(-1.0, 1.0);

    leftPID.SetP(0.5);
    leftPID.SetI(0.0);
    leftPID.SetD(0.0);
    leftPID.SetIZone(0.0);
    leftPID.SetFF(0.0);
    leftPID.SetOutputRange(-1.0, 1.0);

    rightPID.SetP(0.5);
    rightPID.SetI(0.0);
    rightPID.SetD(0.0);
    rightPID.SetIZone(0.0);
    rightPID.SetFF(0.0);
    rightPID.SetOutputRange(-1.0, 1.0);
}

//Flywheels
void Shooter::Shoot(double elevatorHeight, double angleError, double actualAngle)
{
    leftShooterMotor.Set(1.0);
    rightShooterMotor.Set(1.0);
    frc::SmartDashboard::PutNumber("Left Velocity", leftShooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Right Velocity", rightShooterEncoder.GetVelocity());
    if(elevatorEncoder.GetPosition() > elevatorHeight - 5.0 && elevatorEncoder.GetPosition() < elevatorHeight + 5.0)
    {
        IntakeIn();
    }
}

// bool Shooter::IsDone()
// {
//     if(timer.Get() > 5_s)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

void Shooter::StopShooting(bool runWheels)
{
    if(runWheels)
    {
        leftShooterMotor.Set(1.0);
        rightShooterMotor.Set(1.0);
    }
    else
    {
        leftShooterMotor.StopMotor();
        rightShooterMotor.StopMotor();
    }
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

//Elevator Control
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
    runOnceZero = true;
}

void Shooter::AutoElevatorControl(double elevatorHeight)
{
    if(elevatorEncoder.GetPosition() < elevatorHeight - 1.0 || elevatorEncoder.GetPosition() > elevatorHeight + 1.0)
    {
        elevatorPID.SetReference(elevatorHeight, rev::CANSparkMax::ControlType::kPosition);
    }
}

void Shooter::ZeroElevator()
{
    
    if(elevatorLimit.Get())
    {
        elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        StopElevator();
        elevatorEncoder.SetPosition(0.0);
        runOnceZero = true;
    }
    else
    {
        if(runOnceZero)
        {
            elevatorMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
            elevatorMotor.Set(-1.0);
            runOnceZero = false;
        }
    }
}

void Shooter::GetPosition()
{
    frc::SmartDashboard::PutNumber("Encoder Value", elevatorEncoder.GetPosition());
}