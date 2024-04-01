// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonID);
  private final CANSparkMax frontRight = new CANSparkMax(DriveConstants.frontRightID,  MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.frontLeftID,  MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(DriveConstants.backRightID,  MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(DriveConstants.backLeftID,  MotorType.kBrushless);
  
  private final RelativeEncoder frontRightEncoder = frontRight.getEncoder();
  private final RelativeEncoder frontLeftEncoder = frontLeft.getEncoder();
  private final RelativeEncoder backRightEncoder = backRight.getEncoder();
  private final RelativeEncoder backLeftEncoder = backLeft.getEncoder();

  private final DifferentialDrive drive = new DifferentialDrive(frontLeft::set, frontRight::set);

  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.683);

  private final Field2d field = new Field2d();

  private final DifferentialDrivePoseEstimator poseEstimator = 
    new DifferentialDrivePoseEstimator(
      kinematics, 
      gyro.getRotation2d(), 
      frontLeftEncoder.getPosition(), 
      frontRightEncoder.getPosition(), 
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5,0.5,Units.degreesToRadians(30)));

  // Mutable holder for unit-safe values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
    this::voltageDrive, this::log, this);

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), mechanism);

    
  public Drivetrain() {
    //SendableRegistry.addChild(drive, frontLeft);
    //SendableRegistry.addChild(drive, frontRight);

    frontRight.restoreFactoryDefaults();
    frontLeft.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();

    Timer.delay(4);

    drive.setSafetyEnabled(false);

    frontLeft.setInverted(true);
    backLeft.setInverted(true);

    frontRight.setInverted(false);
    backRight.setInverted(false);

    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    frontLeft.setSmartCurrentLimit(40);
    backLeft.setSmartCurrentLimit(40);
    frontRight.setSmartCurrentLimit(40);
    backRight.setSmartCurrentLimit(40);

    frontRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    frontLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);

    frontRightEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    backRightEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    frontLeftEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    backLeftEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);

    frontLeft.burnFlash();
    backLeft.burnFlash();
    frontRight.burnFlash();
    backRight.burnFlash();

    setCoastMode();

    resetEncoders();
    gyro.setYaw(0);

    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());

    SmartDashboard.putData("Drive/Field", field);
    SmartDashboard.putData("Drive/Gyro", gyro); 
  }
  // Tells SysID how to pass voltage to motor controllers
  public void voltageDrive (Measure<Voltage> volts) {
    frontLeft.setVoltage(volts.in(Volts));
    frontRight.setVoltage(volts.in(Volts));
  }
  // Tells SysId how to read voltage, position, velocity
  public void log(SysIdRoutineLog log) {
    double avgVoltage = ((frontLeft.getAppliedOutput() * frontLeft.getBusVoltage()) 
      + (frontRight.getAppliedOutput() * frontRight.getBusVoltage())) / 2.0;

    double avgLinearPos = (frontLeftEncoder.getPosition() + frontRightEncoder.getPosition()) / 2.0;

    double avgLinearVel = (frontLeftEncoder.getVelocity() + frontRightEncoder.getVelocity()) / 2.0;

    log.motor("drivetrain")
    .voltage(m_appliedVoltage.mut_replace(avgVoltage,Volts))
    .linearPosition(m_distance.mut_replace(avgLinearPos, Meters))
    .linearVelocity(m_velocity.mut_replace(avgLinearVel, MetersPerSecond));
  }

  public void FastMode() {
    setCoastMode();
    drive.setMaxOutput(1);
  }

  public void SlowMode() {
    setBrakeMode();
    drive.setMaxOutput(0.3);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
  }

  public void updateOdometry() {
    poseEstimator.update(
      gyro.getRotation2d(), 
      frontLeftEncoder.getPosition(), 
      frontRightEncoder.getPosition());

    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
    if (limelightMeasurement.tagCount >= 2) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      poseEstimator.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
    }
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }

  public void setMaxSpeed(double speed) {
    drive.setMaxOutput(speed);
  }

  public void setBrakeMode() {
    frontRight.setIdleMode(IdleMode.kBrake);
    frontLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    frontRight.setIdleMode(IdleMode.kCoast);
    frontLeft.setIdleMode(IdleMode.kCoast);
    backRight.setIdleMode(IdleMode.kCoast);
    backLeft.setIdleMode(IdleMode.kCoast);
  }

  public void stopMotors() {
    frontLeft.stopMotor();
    frontRight.stopMotor();
  }

  public void invertMotors() {
    frontLeft.setInverted(true);
    backLeft.setInverted(true);

    frontRight.setInverted(false);
    backRight.setInverted(false);
  }

  public void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
  }

  public double getDistance() {
    return (frontLeftEncoder.getPosition() + frontRightEncoder.getPosition()) / 2.0;
  }

  public void setSafetyEnabled(boolean state) {
    drive.setSafetyEnabled(state);
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    drive.feed();
  }

  public void setYaw(double value) {
    gyro.setYaw(value);
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive/Left Voltage", frontLeft.getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Drive/Distance", getDistance());
    SmartDashboard.putNumber("Drive/Gyro Angle", gyro.getAngle());

    //updateOdometry();
    m_odometry.update(gyro.getRotation2d(),
      frontLeftEncoder.getPosition(),
      frontRightEncoder.getPosition());

    field.setRobotPose(m_odometry.getPoseMeters());
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Command runFrontLeft() {
    return this.startEnd(() -> frontLeft.set(0.3), () -> frontLeft.stopMotor());
  }

  public Command runBackLeft() {
    return this.startEnd(() -> backLeft.set(0.3), () -> backLeft.stopMotor());
  }

  public Command runFrontRight() {
    return this.startEnd(() -> frontRight.set(0.3), () -> frontRight.stopMotor());
  }

  public Command runBackRight() {
    return this.startEnd(() -> backRight.set(0.3), () -> backRight.stopMotor());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  /*
   * log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontLeftEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontLeftEncoder.getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRight.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontRightEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontRightEncoder.getVelocity(), MetersPerSecond));
              }
   */
  
}
