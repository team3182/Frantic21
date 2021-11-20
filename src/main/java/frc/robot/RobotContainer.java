// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoStraightener;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import libs.IO.ConsoleController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channnel 0
  public final ConsoleController m_controller = new ConsoleController(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private Trajectory trajectory;

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Generate a trajectory following Ramsete command
   * 
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were 
   * found empirically by using the frc-characterization tool.
   * 
   * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
   */
  private Command generateRamseteCommand(String pathName) {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            DriveConstants.m_feedforward,
            DriveConstants.kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)), //Use for auto strightener? 
        List.of(
            new Translation2d(0.5, 0.25),
            new Translation2d(1.0, -0.25),
            new Translation2d(1.5, 0)
        ),
        new Pose2d(0.0, 0, new Rotation2d(Math.PI)),
        config);

    String trajectoryJSON = "output/" + pathName + ".wpilib.json";//replace test with name of path
    trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        DriveConstants.m_feedforward,
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        DriveConstants.m_leftPIDController,
        DriveConstants.m_rightPIDController,
        m_drivetrain::tankDriveVolts,
        m_drivetrain);

    m_drivetrain.resetOdometry(trajectory.getInitialPose());

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose()), m_drivetrain)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain));
  } 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());
    //m_drivetrain.setDefaultCommand(getTankDriveCommand());

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenInactive(new PrintCommand("Button A Released"));
    
    //Console Controller Mapping 
    m_controller.a
      .whenPressed(generateRamseteCommand("startTeleopPath"));//Run startTeleopPath
    m_controller.b
      .whenPressed(new AutoStraightener(m_drivetrain));//Run Full Path
    m_controller.x
      .whenPressed(new PrintCommand("Button X on Controller Pressed"));//Replace PrintCommand with Command for: Auto Angle Correction Button
    m_controller.y
      .whenPressed(generateRamseteCommand("endTeleopPath"));//Run endTeleopPath
    
    //Extra mapping slots for running PathWeaver Path if needed
    m_controller.start
      .whileHeld(new TankDrive(m_drivetrain, () -> -m_controller.getLeftTrigger() * .9, () -> m_controller.getRightTrigger() * 0));
    m_controller.back
      .whileHeld(new TankDrive(m_drivetrain, () -> -m_controller.getLeftTrigger() * 0, () -> m_controller.getRightTrigger() * .6));
    
    //For Preprogammed turning
    m_controller.topDPAD
      .whenPressed(new TurnDegrees(-45, m_drivetrain));//Replace PrintCommand with Command for: -45 degree turn
    m_controller.bottomDPAD
      .whenPressed(new TurnDegrees(45, m_drivetrain));//Replace PrintCommand with Command for: 45 degree turn
    m_controller.leftDPAD
      .whenPressed(new TurnDegrees(-90, m_drivetrain));//Replace PrintCommand with Command for: -90 degree turn
    m_controller.rightDPAD
      .whenPressed(new TurnDegrees(90, m_drivetrain));//Replace PrintCommand with Command for: 90 degree turn

    //For Tank Drive
    //m_controller.leftBumper
      //.whileHeld(new TankDrive(m_drivetrain, m_controller.leftBumper, 0));

    // Setup SmartDashboard options
    //m_chooser.setDefaultOption("Ramsete Trajectory", generateRamseteCommand("Group2_Nishesh2"));
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));

    Shuffleboard.getTab("SmartDashboard")
    .add("Max Speed", 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
    .getEntry();
    
    SmartDashboard.putData("Autnomous Routine", m_chooser);
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getLeftStickY(), () -> m_controller.getLeftStickX());
    }

  public Command getTankDriveCommand(){
    return new TankDrive(
      m_drivetrain, () -> -m_controller.getLeftStickY(), () -> m_controller.getRightStickY());
  }
}
