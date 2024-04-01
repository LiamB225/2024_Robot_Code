// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>

#include "Constants.h"

class Climber {
 public:
  Climber();

  void PistonsOut();
  void PistonsIn();
  void PistonsToggle();

  frc::DoubleSolenoid ClimberPistons { 
    OperatorConstants::pneumaticsHubID, 
    frc::PneumaticsModuleType::REVPH, 
    OperatorConstants::SolenoidForwardID, 
    OperatorConstants::SolenoidReverseID
  };

};
