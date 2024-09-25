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
#include <frc/Timer.h>
#include <units/time.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

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

  frc::SendableChooser<std::string> m_chooser;
  const std::string kautoLeft = "Left";
  const std::string kautoCenter = "Center";
  const std::string kautoRight = "Right";
  std::string autoSelected;


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
  bool xboxX = false;
  bool manualShoot = false;
  bool manualIntake = false;

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
  frc::Timer Timer;
  frc::DigitalInput Switch { OperatorConstants::SwitchID };
  bool m_switch = false;
  void RedAuto();
  void BlueAuto();
  bool teleop = false;
  const std::vector<double> redCenterPos { 6.928, 1.442593, 180.0 };
  const std::vector<double> redLeftPos { 7.622467, 0.2544062, -120.0 };
  const std::vector<double> redRightPos { 7.622467, 2.630779, 120.0 };
  const std::vector<double> blueCenterPos { -6.928, 11.442593, 0.0 };
  const std::vector<double> blueLeftPos { -7.622467, 2.630779, 60.0 };
  const std::vector<double> blueRightPos { -7.622467, 0.2544062, -60.0 };

  //Drive Train
  void NormalDrive();

  //Compressor
  frc::Compressor Compressor { OperatorConstants::pneumaticsHubID, frc::PneumaticsModuleType::REVPH };
};
