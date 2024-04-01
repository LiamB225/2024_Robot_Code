

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <wpinet/PortForwarder.h>

void Robot::RobotInit()
{
  for (int port = 5800; port <= 5805; port++)
  {
    wpi::PortForwarder::GetInstance().Add(port, "limelight.local", port);
  }

  m_Drive = new Drive();
  m_ATPS = new ATPS();
  m_Shooter = new Shooter();
  m_Climber = new Climber();

  Compressor.EnableDigital();
}

void Robot::RobotPeriodic()
{
  m_Drive->EstimatePosition(m_ATPS->PositionSpeaker());
  currentPos = m_ATPS->PositionSpeaker();
  //frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit()
{
  while(currentPos[0] == 0.0 && currentPos[1] == 0.0 && currentPos[2] == 0.0)
  {
   currentPos = m_ATPS->PositionSpeaker();
  }
  m_Drive->ResetPosition(currentPos);
  m_Drive->SetTarget(5.8, 1.44, 179);
}

void Robot::AutonomousPeriodic()
{
  m_Drive->Track();
  m_Drive->SetVoltages();
}

void Robot::TeleopInit()
{
  m_Climber->PistonsIn();
}

void Robot::TeleopPeriodic()
{
  //Get Xbox Inputs
  GetXbox();

  //Flywheels
  if(xboxY)
  {
    AutoShoot();
  }
  else
  {
    //Intake
    if(m_Shooter->GetIntakeSensor())
    {
      //Elevator
      if(xboxB)
      {
        ElevatorControl();
      }
      else
      {
        m_Shooter->ZeroElevator();
      }
      Intake();
    }
    else
    {
      IntakeLimited();
      EstimateElevatorHeight();
    }
    
    StopAutoShoot();

    //MecanumDrive
    NormalDrive();
  }
  
  //Pistons
  if(xboxA)
  {
    m_Climber->PistonsToggle();
  }
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

void Robot::GetXbox()
{
  //Drive Train Controls
  xboxLX = Xbox.GetLeftX();
  if(xboxLX < .2 && xboxLX > -.2)
  {
    xboxLX = 0;
  }
  xboxLY = Xbox.GetLeftY();
  if(xboxLY < .2 && xboxLY > -.2)
  {
    xboxLY = 0;
  }
  xboxRX = Xbox.GetRightX();
  if(xboxRX < .2 && xboxRX > -.2)
  {
    xboxRX = 0;
  }
  
  //Intake Controls
  if(Xbox.GetRightTriggerAxis() > .1)
  {
    xboxRightTrigger = true;
    xboxLeftTrigger = false;
  }
  else if(Xbox.GetLeftTriggerAxis() > .1)
  {
    xboxLeftTrigger = true;
    xboxRightTrigger = false;
  }
  else
  {
    xboxRightTrigger = false;
    xboxLeftTrigger = false;
  }

  //Shooting Control
  xboxY = Xbox.GetYButton();
  if(Xbox.GetXButtonPressed())
  {
    xboxX =! xboxX;
  }

  //Elevator Control
  xboxRightBumper = Xbox.GetRightBumper();
  xboxLeftBumper = Xbox.GetLeftBumper();
  if(Xbox.GetBButtonPressed())
  {
    xboxB =! xboxB;
  }
  frc::SmartDashboard::PutNumber("B", xboxB);

  //Piston Controls
  xboxA = Xbox.GetAButtonPressed();
}

void Robot::NormalDrive()
{
  m_Drive->Cartesian(-xboxLY, xboxLX, xboxRX);
}

void Robot::AutoShoot()
{
  //auto aiming
  if(currentPos[0] != 0.0 && currentPos[1] != 0.0 && currentPos[2] != 0.0 && runOnceShooter)
  {
    m_Drive->ResetPosition(m_ATPS->PositionSpeaker());
    runOnceShooter = false;
  }
  else
  {
    xboxY = false;
  }
  m_Shooter->Shoot(elevatorHeight, angleError, currentPos[0]);
  m_Drive->TrackTeleop();
  m_Drive->SetVoltages();
  m_Shooter->AutoElevatorControl(elevatorHeight);
  frc::SmartDashboard::PutNumber("elevator Height", m_ATPS->ElevatorHeight());
}

void Robot::StopAutoShoot()
{
  m_Shooter->StopShooting(xboxX);
  m_Drive->SetShooterTarget(m_ATPS->AngleError());
  elevatorHeight = m_ATPS->ElevatorHeight();
  angleError = m_ATPS->AngleError();
  runOnceShooter = true;
}

void Robot::EstimateElevatorHeight()
{
  if(510.0 > m_ATPS->ElevatorHeight() && m_ATPS->ElevatorHeight() > 400.0)
  {
    m_Shooter->AutoElevatorControl(450.0);
  }
  else if(400.0 > m_ATPS->ElevatorHeight() && m_ATPS->ElevatorHeight() > 300.0)
  {
    m_Shooter->AutoElevatorControl(350.0);
  }
  else if(300.0 > m_ATPS->ElevatorHeight() && m_ATPS->ElevatorHeight() > 200.0)
  {
    m_Shooter->AutoElevatorControl(250.0);
  }
  else if(200.0 > m_ATPS->ElevatorHeight() && m_ATPS->ElevatorHeight() > 100.0)
  {
    m_Shooter->AutoElevatorControl(150.0);
  }
  else
  {
    m_Shooter->AutoElevatorControl(50.0);
  }
}

void Robot::Intake()
{
  if(xboxRightTrigger)
  {
    m_Shooter->IntakeIn();
  }
  else if(xboxLeftTrigger)
  {
    m_Shooter->IntakeOut();
  }
  else
  {
    m_Shooter->StopIntake();
  }
}

void Robot::IntakeLimited()
{
  if(xboxLeftTrigger)
  {
    m_Shooter->IntakeOut();
  }
  else
  {
    m_Shooter->StopIntake();
  }
}

void Robot::ElevatorControl()
{
  if(xboxRightBumper)
  {
    m_Shooter->AngleUp();
  }
  else if(xboxLeftBumper)
  {
    m_Shooter->AngleDown();
  }
  else
  {
    m_Shooter->StopElevator();
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
