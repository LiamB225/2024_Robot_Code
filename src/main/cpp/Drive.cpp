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

void Drive::SetTarget(double targetXPos, double targetYPos, double targetRotPos)
{
    pidX.SetGoal((units::meter_t)(targetXPos));
    pidY.SetGoal((units::meter_t)(targetYPos));
    pidRot.SetGoal((units::degree_t)(targetRotPos));
}

void Drive::Track(std::vector<double> currentPos)
{
    double X = pidX.Calculate((units::meter_t)(currentPos[0]));
    double Y = pidY.Calculate((units::meter_t)(currentPos[1]));
    double Rot = pidRot.Calculate((units::degree_t)(currentPos[2]));
    //myMecanumDrive->DriveCartesian(newY, newX, Rot);
    frc::SmartDashboard::PutNumber("ValueX", X);
    frc::SmartDashboard::PutNumber("ValueY", Y);
    frc::SmartDashboard::PutNumber("ValueRot", Rot);
}

void Drive::EndTargeting()
{

}