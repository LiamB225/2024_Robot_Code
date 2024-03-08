

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
}

void Robot::RobotPeriodic()
{
  //frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic()
{
  
}

void Robot::TeleopInit()
{
  m_Drive->SetTarget(0.0, -1.0, 0.0);
}

void Robot::TeleopPeriodic()
{
  GetXbox();
  //std::vector<double> newPos;
  //newPos = m_ATPS->PositionStage();
  //if(newPos[0] == 0.0 && newPos[1] == 0.0 && newPos[2] == 0.0)
  //{
  //  newPos = lastPos;
  //}
  //else
  //{
  //  lastPos = newPos;
  //}
  //m_Drive->Track(newPos);

  //if(xboxRightBumper)
  //{
  //  StageDrive();
  //}
  //else
  //{
      NormalDrive();
  //}

  //Flywheels
  if(xboxY)
  {
    m_Shooter->Shoot();
  }
  else
  {
    m_Shooter->StopShooting();
  }

  //Intake
  if(m_Shooter->GetIntakeSensor())
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
  else if(xboxLeftTrigger)
  {
    m_Shooter->IntakeOut();
  }
  else
  {
    m_Shooter->StopIntake();
  }

  //Elevator
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

  if(xboxB)
  {
    if(m_Shooter->ZeroElevator())
    {
      xboxB = false;
    }
  }
  
  m_Shooter->GetPosition();
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
  if(Xbox.GetYButtonPressed())
  {
    xboxY =! xboxY;
  }

  //Elevator Control
  xboxRightBumper = Xbox.GetRightBumper();
  xboxLeftBumper = Xbox.GetLeftBumper();
  if(Xbox.GetBButtonPressed())
  {
    xboxB =! xboxB;
  }
  frc::SmartDashboard::PutNumber("B", xboxB);
}

void Robot::NormalDrive()
{
  m_Drive->Cartesian(-xboxLY, xboxLX, xboxRX);
}

void Robot::StageDrive()
{

}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
