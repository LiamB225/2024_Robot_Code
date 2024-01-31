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
    return std::vector<double>();
}

std::vector<double> ATPS::PositionStage()
{
    std::vector<double> pos(6);
    pos = ntinst.GetTable("limelight")->GetNumberArray("botpose_targetspace", std::vector<double>(6));
    return std::vector<double> { pos[0], pos[1], pos[5] };
}