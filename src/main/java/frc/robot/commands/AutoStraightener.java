// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoStraightener extends CommandBase {
  /**
   * Creates a new Autonomous Drive based on time. This will drive out for a period of time, turn
   * around for time (equivalent to time to turn around) and drive forward again. This should mimic
   * driving out, turning around and driving back.
   *
   * @param drivetrain The drive subsystem on which this command will run
   */

  private final double heading;
  private final double straightDegrees;
  


  public AutoStraightener(Drivetrain drivetrain) {

    // Gets robot heading
    heading = drivetrain.getHeading();

    /* 
      Rounds the heading divided by 90, and multiplys that rounded number by 90 to get which
      heading is closest, then subtracting it from the heading to get the ammount the robot 
      needs to turn
    */
    

    straightDegrees = heading - (90 * Math.round(heading/90));//consider if heading is 45, or consider buttons for all four compass directions. Also consider putting auto striaghtner into a new class that extends the CommandBase instead of SequentialCommmadGroup
    

    
  }
}
