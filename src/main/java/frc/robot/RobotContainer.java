// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_exampleSubsystem = new DrivetrainSubsystem();

    public DifferentialDriveKinematics diff =  new DifferentialDriveKinematics(.7112);
  private final Drive m_autoCommand = new Drive(m_exampleSubsystem);
    private RamseteCommand ramseteCommand;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
      var autoVoltageConstraint =
              new DifferentialDriveVoltageConstraint(
                      new SimpleMotorFeedforward(
                              .55834, // ks volt
                              3.3207, // kv
                              0.51012), // ka
                      diff, //
                      7);

      // Create config for trajectory
      TrajectoryConfig config =
              new TrajectoryConfig(
                      0.5,
                      0.5)
                      // Add kinematics to ensure max speed is actually obeyed
                      .setKinematics(diff)
                      // Apply the voltage constraint
                      .addConstraint(autoVoltageConstraint)
                      ;

      // An example trajectory to follow.  All units in meters.
      // create a new trajectory 1 meter forward
      Trajectory exampleTrajectory = PathPlanner.loadPath("Auto Routine 1 Part 1", 0.5, 0.5);

      this.ramseteCommand = new RamseteCommand(
              exampleTrajectory,
              m_exampleSubsystem::getPose,
              new RamseteController(2, 0.7),
              new SimpleMotorFeedforward(
                      .55834, // ks volt
                      3.3207, // kv
                      0.51012),
              diff,
              m_exampleSubsystem::getWheelSpeeds,
              new PIDController(4.5729, 0, 0),
              new PIDController(4.5729, 0, 0),
              // RamseteCommand passes volts to the callback
              m_exampleSubsystem::tankDriveVolts,
              m_exampleSubsystem);
      m_exampleSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> m_exampleSubsystem.tankDriveVolts(0, 0));
  }
}
