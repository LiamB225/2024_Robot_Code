// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>
#include <frc/Compressor.h>
#include <vector>

#include "Drive.h"
#include "Constants.h"
#include "ATPS.h"
#include "Shooter.h"
#include "Climber.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  //Pointers
  Drive *m_Drive;
  ATPS *m_ATPS;
  Shooter *m_Shooter;
  Climber *m_Climber;

  //Xbox Controller
  void GetXbox();
  frc::XboxController Xbox { OperatorConstants::kDriverControllerPort };
  double xboxLX = 0;
  double xboxLY = 0;
  double xboxRX = 0;
  bool xboxRightTrigger = false;
  bool xboxLeftTrigger = false;
  bool xboxY = false;
  bool xboxRightBumper = false;
  bool xboxLeftBumper = false;
  bool xboxB = false;
  bool xboxA = false;
  bool xboxX = true;

  //auto shooting
  bool runOnceShooter = true;
  std::vector<double> currentPos;
  double elevatorHeight = 0.0;
  double angleError = 0.0;
  void AutoShoot();
  void StopAutoShoot();
  void EstimateElevatorHeight();
  void Intake();
  void IntakeLimited();
  void ElevatorControl();

  //Drive Train
  void NormalDrive();

  //Compressor
  frc::Compressor Compressor { OperatorConstants::pneumaticsHubID, frc::PneumaticsModuleType::REVPH };
};
