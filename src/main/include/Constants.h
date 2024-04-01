// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
const int limitSwitchID = 0;
const int intakeSensorID = 1;
const int frontLeftID = 1;
const int frontRightID = 2;
const int backLeftID = 3;
const int backRightID = 4;
const int leftShooterID = 5;
const int rightShooterID = 6;
const int topIntakeID = 7;
const int bottomIntakeID = 8;
const int elevatorID = 9;
const int pneumaticsHubID = 10;
const int SolenoidForwardID = 1;
const int SolenoidReverseID = 0;

}  // namespace OperatorConstants
