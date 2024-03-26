// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

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

  private final DifferentialDrivePoseEstimator poseEstimator = 
    new DifferentialDrivePoseEstimator(
      kinematics, 
      gyro.getRotation2d(), 
      frontLeftEncoder.getPosition(), 
      frontRightEncoder.getPosition(), 
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5,0.5,Units.degreesToRadians(30)));

  public Drivetrain() {
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

    frontLeft.burnFlash();
    backLeft.burnFlash();
    frontRight.burnFlash();
    backRight.burnFlash();

    setCoastMode();

    resetEncoders();
    gyro.setYaw(0);
    
    frontRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    frontLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);

    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
  }

  public void FastMode() {
    setCoastMode();
    drive.setMaxOutput(1);
  }

  public void SlowMode() {
    setBrakeMode();
    drive.setMaxOutput(0.3);
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

  public void setSafteyEnabled(boolean state) {
    drive.setSafetyEnabled(state);
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void setYaw(double value) {
    gyro.setYaw(value);
  }
/*
  public void arcadeDriveManual(double speed, double rotation) {
    if (speed < 0.1 && speed > -0.1) {
      speed = 0;
    }
    if (rotation < 0.1 && rotation > -0.1) {
      rotation = 0;
    }

      frontLeft.set(speed + rotation);
      frontRight.set(speed - rotation);
  }
*/
  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    drive.feed();
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Drive Encoder", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Right Drive Encoder", frontRightEncoder.getPosition());
    SmartDashboard.putNumber("Distance", getDistance());
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());

    //updateOdometry();
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
  
}
