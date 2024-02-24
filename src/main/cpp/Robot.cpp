

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
}

void Robot::RobotPeriodic()
{
  //frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  m_Drive->SetTarget(0.0, -1.0, 0.0);
}

void Robot::TeleopPeriodic()
{
  GetXbox();
  std::vector<double> newPos;
  newPos = m_ATPS->PositionStage();
  if(newPos[0] == 0.0 && newPos[1] == 0.0 && newPos[2] == 0.0)
  {
    newPos = lastPos;
  }
  else
  {
    lastPos = newPos;
  }
  m_Drive->Track(newPos);
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

void Robot::GetXbox()
{
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

  xboxRightBumper = Xbox.GetRightBumper();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
