// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  static int FORWARD45 = 45;
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    addCommands(
      new DriveDistance(1, 15, drivetrain, 9999),

      new TurnDegrees(105, drivetrain), // original 90
      new DriveDistance(1, 16.5, drivetrain, 105),

      new TurnDegrees(-75, drivetrain), // original -90
      new DriveDistance(1, 16.5, drivetrain, -75),
      
      new TurnDegrees(-45, drivetrain),
      new DriveDistance(1, 15*Math.sqrt(2)-1, drivetrain, -45),

      new TurnDegrees(0, drivetrain),
      new DriveDistance(1, 7.0, drivetrain, 0), // adjusted distance from 7.5 to 7.0

      new TurnDegrees(92.5, drivetrain), // adjusted turn degrees from 90 to 91
      new DriveDistance(1, 31, drivetrain, 92.5), // adjusted turn degrees from 90

      new TurnDegrees(-80, drivetrain), // adjusted from 90
      new DriveDistance(1, 31, drivetrain, -80),

      new TurnDegrees(5, drivetrain),
      new DriveDistance(1, 22.5, drivetrain, 5), 

      new TurnDegrees(102.5, drivetrain), // adjusted from 90
      new DriveDistance(1, 31, drivetrain, 102.5),

      new TurnDegrees(-82.5, drivetrain),
      new DriveDistance(1, 14, drivetrain, -82.5),

      new TurnDegrees(0, drivetrain),
      new DriveDistance(1, 15, drivetrain, 0)
    );
  //   addCommands(
  //     // 15 in
  //     new DriveDistance(1, 15, drivetrain, 9999),
  //     // cc 90 deg
  //     new TurnDegrees(90, drivetrain),
  //     // 15 in
  //     new DriveDistance(1, 15, drivetrain, 90),
  //     // c 90 deg
  //     new TurnDegrees(0, drivetrain),
  //     // 45 in
  //     new DriveDistance(1, FORWARD45, drivetrain, 0),
  //     // c 90 deg
  //     new TurnDegrees(-90, drivetrain),
  //     // 15 in
  //     new DriveDistance(1, 10, drivetrain, -90),
  //     // cc 90 deg
  //     new TurnDegrees(0, drivetrain),
  //     // 15 in
  //     new DriveDistance(1, 15, drivetrain, 0),
  //     // cc 90 deg
  //     new TurnDegrees(90, drivetrain),
  //     // 15 in
  //     new DriveDistance(1, 15, drivetrain, 90),
  //     // cc 90 deg
  //     new TurnDegrees(180, drivetrain),
  //     // 15 in
  //     new DriveDistance(1, 15, drivetrain, 180),
  //     // cc 90 deg
  //     new TurnDegrees(270, drivetrain),
  //     // 15 in
  //     new DriveDistance(1, 15, drivetrain, 270),
  //     // c 90 deg
  //     new TurnDegrees(180, drivetrain),
  //     // 45 in
  //     new DriveDistance(1, FORWARD45, drivetrain, 180),
  //     // c 90 deg
  //     new TurnDegrees(90, drivetrain),
  //     // 15 in
  //     new DriveDistance(1, 10, drivetrain, 90),
  //     // cc 90 deg
  //     new TurnDegrees(180, drivetrain),
  //     // 15 in
  //     new DriveDistance(1, 15, drivetrain, 180)

  //     // new DriveDistance(1, 10, drivetrain),
  //     // new TurnDegrees(-TURN90, drivetrain),
  //     // new DriveDistance(1, 10, drivetrain),
  //     // new TurnDegrees(-TURN90, drivetrain),
  //     // new DriveDistance(1, 10, drivetrain),
  //     // new TurnDegrees(-TURN90, drivetrain),
  //     // new DriveDistance(1, 10, drivetrain),
  //     // new TurnDegrees(-TURN90, drivetrain)

  //   );
  }
}
