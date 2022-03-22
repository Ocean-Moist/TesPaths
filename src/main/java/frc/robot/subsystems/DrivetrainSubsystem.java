// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
   // Setup autonomous and sensor objects
   ChassisSpeeds chassisSpeeds;
   DifferentialDriveOdometry odometry;

   private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

   // Setup drive objects
   public final Encoder rightEncoder;
   public final Encoder leftEncoder;
   private final MotorControllerGroup leftMotors;
   private final MotorControllerGroup rightMotors;
   private final DifferentialDrive drive;
   
  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    WPI_TalonFX[] driveMotors = {
      new WPI_TalonFX(1),
      new WPI_TalonFX(2),
      new WPI_TalonFX(4),
      new WPI_TalonFX(3)
    };
    driveMotors[0].setNeutralMode(NeutralMode.Brake);
    driveMotors[1].setNeutralMode(NeutralMode.Brake);
    driveMotors[2].setNeutralMode(NeutralMode.Brake);
    driveMotors[3].setNeutralMode(NeutralMode.Brake);
    leftMotors = new MotorControllerGroup(driveMotors[0], driveMotors[1]);
    rightMotors = new MotorControllerGroup(driveMotors[2], driveMotors[3]);
    rightMotors.setInverted(true);
    
    drive = new DifferentialDrive(leftMotors, rightMotors); // Initialize Differential Drive
    rightEncoder = new Encoder(2, 3, true);
    leftEncoder = new Encoder(0, 1, false); 
    leftEncoder.setDistancePerPulse(2*3.14*(.1524/2)/2048);
    rightEncoder.setDistancePerPulse(2*3.14*(.1524/2)/2048);
    resetEncoders();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d()); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    SmartDashboard.putNumber("left", leftEncoder.getDistance());
    SmartDashboard.putNumber("right", rightEncoder.getDistance());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
  
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
