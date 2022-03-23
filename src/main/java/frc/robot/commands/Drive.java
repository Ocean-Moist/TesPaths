// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.List;

import com.pathplanner.lib.PathPlanner;


public class Drive extends CommandBase {
    private final RamseteCommand ramseteCommand;
    public DifferentialDriveKinematics diff =  new DifferentialDriveKinematics(.7112);
    public Drive(DrivetrainSubsystem drivetrainSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(drivetrainSubsystem);

        // Create a voltage constraint to ensure we don't accelerate too fast
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
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        // create a new trajectory 1 meter forward
        Trajectory exampleTrajectory =         TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        this.ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                drivetrainSubsystem::getPose,
                new RamseteController(2, 0.7),
                new SimpleMotorFeedforward(
                        .55834, // ks volt
                        3.3207, // kv
                        0.51012),
                diff,
                drivetrainSubsystem::getWheelSpeeds,
                new PIDController(4.5729, 0, 0),
                new PIDController(4.5729, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrainSubsystem::tankDriveVolts,
                drivetrainSubsystem);

        // Reset odometry to the starting pose of the trajectory.
        drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
        // Run path following command, then stop at the end.
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        ramseteCommand.initialize();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        ramseteCommand.execute();
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return ramseteCommand.isFinished();
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        ramseteCommand.end(interrupted);
    }
}