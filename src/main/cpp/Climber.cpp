// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Climber.h"

Climber::Climber() = default;

void Climber::PistonsOut()
{
    ClimberPistons.Set(frc::DoubleSolenoid::kForward);
}

void Climber::PistonsIn()
{
    ClimberPistons.Set(frc::DoubleSolenoid::kReverse);
}

void Climber::PistonsToggle()
{
    ClimberPistons.Toggle();
}
