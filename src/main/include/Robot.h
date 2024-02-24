// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>
#include <vector>

#include "Drive.h"
#include "Constants.h"
#include "ATPS.h"

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
  Drive *m_Drive;
  ATPS *m_ATPS;

  std::vector<double> lastPos { 0.0, 0.0, 0.0 };

  frc::XboxController Xbox { OperatorConstants::kDriverControllerPort };
  double xboxLX = 0;
  double xboxLY = 0;
  double xboxRX = 0;
  bool xboxRightBumper = false;

  void GetXbox();
};
