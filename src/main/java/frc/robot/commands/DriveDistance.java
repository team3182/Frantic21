// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;
  private double heading;
  private boolean reset;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double inches, Drivetrain drive, double heading) {
    m_distance = inches;
    m_speed = speed * 0.5;
    m_drive = drive;
    if (heading == 9999) {
      this.heading = 0;
      this.reset = true;
    } else {
      this.heading = heading;
      this.reset = false;
    }
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    if (this.reset) {
      this.m_drive.resetGyro();
    }
    m_drive.resetEncoders();
    System.out.println("Drive " + this.m_distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceFrac = getDistIn() / this.m_distance;
    double rampedSpeed;
    double rampUpFrac = 0.2;
    double rampDownFrac = 0.4;
    if (distanceFrac < rampUpFrac) {
      rampedSpeed = m_speed * (distanceFrac / rampUpFrac);
    } else if (distanceFrac > (1 - rampDownFrac)) {
      rampedSpeed = m_speed * Math.pow((1 - distanceFrac) / rampDownFrac, 3);
    } else {
      rampedSpeed = m_speed;
    }

    double minSpeed = 0.18;
    if (rampedSpeed < minSpeed && rampedSpeed > -minSpeed) {
      if (m_speed > 0) {
        rampedSpeed = minSpeed;
      } else {
        rampedSpeed = -minSpeed;
      }
    }

    double diff = headingDiff(this.m_drive.getHeading(), heading);
    double turnAdjust = diff * -0.01;

    System.out.println("speed " + m_speed + " rampedSpeed " + rampedSpeed + " turnAdjust " + turnAdjust);

    m_drive.m_diffDrive.arcadeDrive(rampedSpeed, turnAdjust, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return (getDistIn()) >= m_distance - 0.2;
  }

  public double getDistIn() {
    return m_drive.getAverageDistanceMeter()*39.3701;
  }

  /**
   * Starting at A, how much rotation to get to B
   */
  public static double headingDiff(double a, double b) {
    if ((a < 0 && b < 0) || (a >= 0 && b >= 0)) {
      // ex 20, 10 -> -10
      return b - a;
    } else {
      double mult;
      if (a > b) {
        mult = 1;
      } else {
        mult = -1;
        double tmp = a;
        a = b;
        b = tmp;
      }
      // a is larger

      // option 1: loop around at +180/-180 (ex +170 -160 -> +30)
      double d1 = (b + 360) - a;
      // option 2: at 0 (ex +10 -20 -> -30)
      double d2 = b - a;
      if (Math.abs(d1) < Math.abs(d2)) {
        return d1 * mult;
      } else {
        return d2 * mult;
      }
    }
  }
}
