# 2024 Robot Code - FRC Team 2143 Team Tobor

This repository contains the C++ robot code for the 2024 FIRST Robotics Competition (FRC) season game, **CRESCENDO**.

## 🤖 Robot Overview

The robot is built using a **Time-Based framework** (Iterative/TimedRobot) and includes the following subsystems:

### Subsystems

*   **Drive**: Mecanum drive drivetrain using REV Spark Max motor controllers.
    *   Features Cartesian control (Forward/Strafe/Turn) and field positioning logic.
*   **Shooter**: Manipulator for handling Note game pieces.
    *   Includes Flywheel control for shooting and Intake rollers for collecting Notes.
    *   Integrates with **ATPS** for auto-aiming based on speaker position.
*   **Elevator**: Mechanism for adjusting the angle/height of the shooter.
    *   Manual control via bumpers or automated control based on distance.
*   **Climber**: Pneumatic piston-based mechanism for endgame climbing.
*   **ATPS**: Automated Targeting & Positioning System for tracking field elements and auto-aligning.

### Hardware

*   **Controllers:** 1x Xbox Controller (Driver)
*   **Sensors:** 
    *   ADIS16470 IMU
    *   Limelight 2 Vision Camera (for ATPS/Pose Estimation)
    *   Intake Sensor (DigitalInput)
    *   Elevator Limit Switch
*   **Motors:**
    *   REV Spark Max Motor Controllers
    *   REV NEO Brushless Motors
    *   REV NEO 550 Brushless Motors

## 🎮 Controller Mappings

### Driver Controller (Port 0)

| Input | Action |
| :--- | :--- |
| **Left Stick Y** | Translation Forward/Backward |
| **Left Stick X** | Translation Left/Right (Strafe) |
| **Right Stick X** | Rotation |
| **Right Trigger** | Intake In |
| **Left Trigger** | Intake Out |
| **Y Button** | Auto Shoot (Aim & Spin Up) |
| **X Button** | Toggle Flywheel Idle State |
| **B Button** | Toggle Elevator Control Mode (Manual vs Zero) |
| **A Button** | Toggle Climber Pistons |
| **Right Bumper** | Elevator Angle Up (When Manual Mode Active) |
| **Left Bumper** | Elevator Angle Down (When Manual Mode Active) |

## 🏹 Autonomous

This project uses custom autonomous logic located in `AutonomousPeriodic`:
*   Auto-alignment to speaker target using ATPS.
*   Drive tracking loop (`m_Drive->Track()`).
*   Initial positioning set via `RobotInit`/`AutonomousInit`.

## 🛠️ Build and Deploy

### Prerequisites
*   WPILib 2024.1.1 or later
*   Visual Studio Code with WPILib extension

### Building the Code
Open the project in VS Code and run the **WPILib: Build Robot Code** command (Shift+F5) or use the terminal:
```bash
./gradlew build
```

### Deploying to Robot
1.  Connect to the robot via USB or Wi-Fi.
2.  Run the **WPILib: Deploy Robot Code** command (Shift+F5) or use the terminal:
```bash
./gradlew deploy
```

---
*Author: Liam Bauer*
