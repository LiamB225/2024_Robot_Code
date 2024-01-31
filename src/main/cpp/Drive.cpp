// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drive.h"

Drive::Drive()
{
    myMecanumDrive = new frc::MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
}

void Drive::Cartesian(double drivePower, double strafePower, double turnPower)
{
    myMecanumDrive->DriveCartesian(drivePower, strafePower, turnPower);
}

void Drive::setTarget(std::vector<double> targetPos, std::vector<double> startPos)
{
    pidX.SetGoal((units::meter_t)(targetPos[0]));
    pidY.SetGoal((units::meter_t)(targetPos[1]));
    pidRot.SetGoal((units::degree_t)(targetPos[2]));
}

void Drive::Track(std::vector<double> currentPos)
{
    int X = pidX.Calculate((units::meter_t)(currentPos[0]));
    int Y = pidY.Calculate((units::meter_t)(currentPos[1]));
    int Rot = pidRot.Calculate((units::degree_t)(currentPos[2]));
    int newX = X * units::math::cos((units::degree_t)(currentPos[2])) - Y * units::math::sin((units::degree_t)(currentPos[2]));
    int newY = X * units::math::sin((units::degree_t)(currentPos[2])) + Y * units::math::cos((units::degree_t)(currentPos[2]));
    myMecanumDrive->DriveCartesian(newY, newX, Rot);
}

void Drive::endTargeting()
{

}