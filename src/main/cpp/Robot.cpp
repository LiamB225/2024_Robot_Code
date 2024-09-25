

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
  Timer.Start();
  m_chooser.SetDefaultOption(kautoCenter, kautoCenter);
  m_chooser.AddOption(kautoLeft, kautoLeft);
  m_chooser.AddOption(kautoRight, kautoRight);
  frc::SmartDashboard::PutData("Auto Mode", &m_chooser);
}

void Robot::RobotPeriodic()
{
  m_switch = Switch.Get();
  frc::SmartDashboard::PutBoolean("switch", m_switch);
  m_Drive->EstimatePosition(m_ATPS->PositionSpeaker(m_switch, m_Drive->GetEstimateX(), m_Drive->GetEstimateY()));
  currentPos = m_ATPS->PositionSpeaker(m_switch, m_Drive->GetEstimateX(), m_Drive->GetEstimateY());
  //frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit()
{
  autoSelected = m_chooser.GetSelected();
  if(m_switch)
  {
    if(autoSelected == kautoCenter)
    {
      m_Drive->ResetPosition(redCenterPos);
    }
    else if(autoSelected == kautoLeft)
    {
      m_Drive->ResetPosition(redLeftPos);
    }
    else
    {
      m_Drive->ResetPosition(redRightPos);
    }
  }
  else
  {
    if(autoSelected == kautoCenter)
    {
      m_Drive->ResetPosition(blueCenterPos);
    }
    else if(autoSelected == kautoLeft)
    {
      m_Drive->ResetPosition(blueLeftPos);
    }
    else
    {
      m_Drive->ResetPosition(blueRightPos);
    }
  }
  m_Drive->SetTarget(5.0, 1.442593, 180.0);
  Timer.Reset();
  xboxX = true;
}

void Robot::AutonomousPeriodic()
{
  frc::SmartDashboard::PutNumber("Time", Timer.Get().value());
  if(m_switch)
  {
    RedAuto();
  }
  else
  {
    BlueAuto();
  }
}

void Robot::TeleopInit()
{
  m_Climber->PistonsIn();
  xboxX = false;
}

void Robot::TeleopPeriodic()
{
  //Get Xbox Inputs
  GetXbox();
  teleop = true;

  //Flywheels
  if(xboxY)
  {
    AutoShoot();
  }
  else if(manualShoot)
  {
    m_Shooter->AutoElevatorControl(500.0);
    m_Shooter->Shoot(500.0, 0.0, 0.0);
  }
  else
  {
    //Intake
    if(m_Shooter->GetIntakeSensor() || manualIntake)
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
      if(xboxB)
      {
        ElevatorControl();
      }
      else
      {
        m_Shooter->ZeroElevator();
      }
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
  frc::SmartDashboard::PutBoolean("B", xboxB);
  frc::SmartDashboard::PutBoolean("X", xboxX);

  //Piston Controls
  xboxA = Xbox.GetAButtonPressed();
  manualShoot = Xbox.GetStartButton();
  if(Xbox.GetBackButtonPressed())
  {
    manualIntake =! manualIntake;
  }
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
    m_Drive->ResetPosition(m_ATPS->PositionSpeaker(m_switch, m_Drive->GetEstimateX(), m_Drive->GetEstimateY()));
    runOnceShooter = false;
  }
  else
  {
    xboxY = false;
  }
  m_Shooter->Shoot(elevatorHeight, angleError, currentPos[2]);
  m_Drive->TrackTeleop();
  m_Drive->SetVoltages();
  m_Shooter->AutoElevatorControl(elevatorHeight);
  frc::SmartDashboard::PutNumber("elevator Height", m_ATPS->ElevatorHeight());
}

void Robot::StopAutoShoot()
{ 
  if(teleop)
  {
    m_Shooter->StopShooting(xboxX);
  }
  else
  {
    m_Shooter->StopShooting(true);
  }
  m_Drive->SetShooterTarget(m_ATPS->AngleError(m_switch));
  elevatorHeight = m_ATPS->ElevatorHeight();
  angleError = m_ATPS->AngleError(m_switch);
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

void Robot::RedAuto()
{
  // if(Timer.Get() > 13.75_s)
  // {
  //   AutoShoot();
  // }
  // else if(Timer.Get() > 11.5_s)
  // {
  //   StopAutoShoot();
  //   m_Drive->Track();
  //   m_Drive->SetVoltages();
  //   m_Shooter->AutoElevatorControl(400.0);
  //   if(m_Shooter->GetIntakeSensor())
  //   {
  //     m_Shooter->IntakeIn();
  //   }
  //   else
  //   {
  //     m_Shooter->StopIntake();
  //   }
  // }
  // else if(Timer.Get() > 11.4_s)
  // {
  //   m_Drive->SetTarget(6.7, 2.13, 158.0);
  //   m_Shooter->ZeroElevator();
  // }
  // else if(Timer.Get() > 9.5_s)
  // {
  //   StopAutoShoot();
  //   m_Drive->Track();
  //   m_Drive->SetVoltages();
  //   m_Shooter->ZeroElevator();
  //   if(m_Shooter->GetIntakeSensor())
  //   {
  //     m_Shooter->IntakeIn();
  //   }
  //   else
  //   {
  //     m_Shooter->StopIntake();
  //   }
  // }
  // else if(Timer.Get() > 8.25_s)
  // {
  //   m_Shooter->ZeroElevator();
  // }
  if(autoSelected == kautoCenter)
  {
    if(Timer.Get() > 8.0_s)
    {
      //m_Drive->SetTarget(5.8, 2.89, 158.0);
      AutoShoot();
    }
    else if(Timer.Get() > 5.0_s)
    {
      StopAutoShoot();
      m_Drive->Track();
      m_Drive->SetVoltages();
      m_Shooter->AutoElevatorControl(400.0);
      if(m_Shooter->GetIntakeSensor())
      {
        m_Shooter->IntakeIn();
      }
      else
      {
        m_Shooter->StopIntake();
      }
    }
    else if (Timer.Get() > 4.9_s)
    {
      m_Drive->SetTarget(6.7, 1.442593, 180.0);
      m_Shooter->ZeroElevator();
    }
    else if(Timer.Get() > 3.0_s)
    {
      StopAutoShoot();
      m_Drive->Track();
      m_Drive->SetVoltages();
      m_Shooter->ZeroElevator();
      if(m_Shooter->GetIntakeSensor())
      {
        m_Shooter->IntakeIn();
      }
      else
      {
        m_Shooter->StopIntake();
      }
    }
    else if(Timer.Get() > 1.75_s)
    {
      m_Drive->SetTarget(5.5, 1.442593, 180.0);
      m_Shooter->ZeroElevator();
    }
  }
  else if(autoSelected == kautoLeft)
  {
    if(Timer.Get() > 3.0_s)
    {
      StopAutoShoot();
      m_Drive->Track();
      m_Drive->SetVoltages();
      m_Shooter->ZeroElevator();
    }
    else if(Timer.Get() > 1.75_s)
    {
      m_Drive->SetTarget(5.5, -1.5, -135.0);
      m_Shooter->ZeroElevator();
    }
  }
  else
  {
    if(Timer.Get() > 3.0_s)
    {
      StopAutoShoot();
      m_Drive->Track();
      m_Drive->SetVoltages();
      m_Shooter->ZeroElevator();
    }
    else if(Timer.Get() > 1.75_s)
    {
      m_Drive->SetTarget(5.5, 1.442593, 180.0);
      m_Shooter->ZeroElevator();
    }
  }
  
  if(Timer.Get() > 1_s)
  {
    m_Shooter->IntakeIn();
  }
  else
  {
    StopAutoShoot();
  }
}

void Robot::BlueAuto()
{
  // if(Timer.Get() > 13.75_s)
  // {
  //   AutoShoot();
  // }
  // else if(Timer.Get() > 11.5_s)
  // {
  //   StopAutoShoot();
  //   m_Drive->Track();
  //   m_Drive->SetVoltages();
  //   m_Shooter->AutoElevatorControl(400.0);
  //   if(m_Shooter->GetIntakeSensor())
  //   {
  //     m_Shooter->IntakeIn();
  //   }
  //   else
  //   {
  //     m_Shooter->StopIntake();
  //   }
  // }
  // else if(Timer.Get() > 11.4_s)
  // {
  //   m_Drive->SetTarget(-6.7, 2.13, 22.0);
  //   m_Shooter->ZeroElevator();
  // }
  // else if(Timer.Get() > 9.5_s)
  // {
  //   StopAutoShoot();
  //   m_Drive->Track();
  //   m_Drive->SetVoltages();
  //   m_Shooter->ZeroElevator();
  //   if(m_Shooter->GetIntakeSensor())
  //   {
  //     m_Shooter->IntakeIn();
  //   }
  //   else
  //   {
  //     m_Shooter->StopIntake();
  //   }
  // }
  // else if(Timer.Get() > 8.25_s)
  // {
  //   m_Shooter->ZeroElevator();
  // }
  if(autoSelected == kautoCenter)
  {
    if(Timer.Get() > 8.0_s)
    {
      //m_Drive->SetTarget(-5.7, 2.89, 50.0);
      AutoShoot();
    }
    else if(Timer.Get() > 5.0_s)
    {
      StopAutoShoot();
      m_Drive->Track();
      m_Drive->SetVoltages();
      m_Shooter->AutoElevatorControl(400.0);
      if(m_Shooter->GetIntakeSensor())
      {
        m_Shooter->IntakeIn();
      }
      else
      {
        m_Shooter->StopIntake();
      }
    }
    else if (Timer.Get() > 4.9_s)
    {
      m_Drive->SetTarget(-6.7, 1.442593, 0.0);
      m_Shooter->ZeroElevator();
    }
    else if(Timer.Get() > 3.0_s)
    {
      StopAutoShoot();
      m_Drive->Track();
      m_Drive->SetVoltages();
      m_Shooter->ZeroElevator();
      if(m_Shooter->GetIntakeSensor())
      {
        m_Shooter->IntakeIn();
      }
      else
      {
        m_Shooter->StopIntake();
      }
    }
    else if(Timer.Get() > 1.75_s)
    {
      m_Drive->SetTarget(-5.5, 1.442593, 0.0);
      m_Shooter->ZeroElevator();
    }
  }
  else if(autoSelected == kautoLeft)
  {
    if(Timer.Get() > 3.0_s)
    {
      StopAutoShoot();
      m_Drive->Track();
      m_Drive->SetVoltages();
      m_Shooter->ZeroElevator();
    }
    else if(Timer.Get() > 1.75_s)
    {
      m_Drive->SetTarget(5.5, -1.5, -135.0);
      m_Shooter->ZeroElevator();
    }
  }
  else
  {
    if(Timer.Get() > 3.0_s)
    {
      StopAutoShoot();
      m_Drive->Track();
      m_Drive->SetVoltages();
      m_Shooter->ZeroElevator();
    }
    else if(Timer.Get() > 1.75_s)
    {
      m_Drive->SetTarget(-5.5, -1.5, -45.0);
      m_Shooter->ZeroElevator();
    }
  }
  
  
  if(Timer.Get() > 1_s)
  {
    m_Shooter->IntakeIn();
  }
  else
  {
    StopAutoShoot();
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
