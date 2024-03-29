// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ATPS.h"

ATPS::ATPS()
{
    ntinst = nt::NetworkTableInstance::GetDefault();

}

std::vector<double> ATPS::PositionSpeaker()
{
    std::vector<double> pos(6);
    pos = ntinst.GetTable("limelight")->GetNumberArray("botpose", std::vector<double>(6));
    frc::SmartDashboard::PutNumber("X", pos[0]);
    frc::SmartDashboard::PutNumber("Y", pos[1]);
    frc::SmartDashboard::PutNumber("Rot", pos[5]);
    distance = hypot((pos[0] - redCenterX), (pos[1] - redCenterY));
    frc::SmartDashboard::PutNumber("Distance", distance);
    return std::vector<double> { pos[0], pos[1], pos[5] };
}

double ATPS::ElevatorHeight()
{
    
}