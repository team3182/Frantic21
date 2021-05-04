// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegrees extends CommandBase {
  private final Drivetrain m_drive;

  private static double endingAngle; 

  private double slowDownVariable; 
  private double m_degrees;
  private double currentDegrees;
  private double finalSpeed;

  private double m_speed = 0.5; 

  private boolean reset;


  public TurnDegrees(double degrees, Drivetrain train) {
    this(degrees, train, false);
  }

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double degrees, Drivetrain drive, boolean reset) {
    m_degrees = degrees;
    m_drive = drive;
    this.reset = reset;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    if (reset) {
      m_drive.resetGyro();
    }

    //m_degrees -= ((endingAngle == 0) ? 0 : (endingAngle - m_degrees));
    System.out.println("Degrees " + m_degrees);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDegrees = getHeading();
    double diff = DriveDistance.headingDiff(currentDegrees, this.m_degrees);

    slowDownVariable = Math.pow(diff / 180, 1);

    finalSpeed = m_speed * slowDownVariable;
    double minSpeed = 0.12;
    if (Math.abs(finalSpeed) < minSpeed) {
      if (finalSpeed < 0) {
        finalSpeed = -minSpeed;
      } else {
        finalSpeed = minSpeed;
      }
    }
  

    m_drive.m_diffDrive.arcadeDrive(0, -finalSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    endingAngle = getHeading();

    System.out.println("Ending Angle (at end): " + endingAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
    // Compare distance travelled from start to distance based on degree turn
    return Math.abs(DriveDistance.headingDiff(getHeading(), this.m_degrees)) < 1;
  }

  private double getHeading() {
    return m_drive.getHeading();
  }
}
