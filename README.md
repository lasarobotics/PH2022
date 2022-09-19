# FRC 2022
Team 4561's 2022 robot code. Tempest's code is written in Java and is based off of WPILib's Java control system.

The code is organised into several subsystems, each responsible for a different aspect of the robot's functionality. This document explains this code.

## [Subsystems](src/main/java/frc/robot/subsystems)
### [Drive Subsystem](src/main/java/frc/robot/subsystems/DriveSubsystem.java)
The drive subsystem controls the drivetrain of the robot, which is a 4-wheel mecanum drive. Each side of the robot is driven by a set of 2 NEO brushless motors, providing ample power for our robot.
The NEO also provides the benefit of having a built-in encoder to accuately measure wheel rotations. The motors are connected to the wheels by a gearbox with a 7.64:1 gear ratio.
Lastly, the drivetrain also implements odometry, which allows the robot to use the IMU and encoder data to accuately determine its position on the field, a useful feature for the autonomous period.

Key points:
* 4 wheel mecanum drive
* Powered by 4 NEO brushless motors
* Odometry for accurate positioning during autonomous
