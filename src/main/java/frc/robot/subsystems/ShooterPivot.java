// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.SparkMax;
// import com.revrobotics.SparkAbsoluteEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

// import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterPivot extends SubsystemBase {
  private final SparkMax pivot;
  private final SparkMaxConfig pivotConfig;
  private final SparkAbsoluteEncoder thruBore;
  private final DigitalInput pivotBottomLimit;
  

  private final TrapezoidProfile pivotProfile;
  private TrapezoidProfile.State pivotStart, pivotSetpoint;
  // private final SparkClosedLoopController pivotPID;
  private final ArmFeedforward pivotFF;

  private double pivotFFValue, lastPivotVelocitySetpoint;

  // Mutable holder for unit-safe values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_angle = Rotations.mutable(0);
  private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

  private final SysIdRoutine.Mechanism pivotmechanism = new SysIdRoutine.Mechanism(this::voltageDrive, this::log, this);

  private final SysIdRoutine pivotCharacterizer;

  //private final SysIdRoutine oldpivotCharacterizer;

  private Rotation2d pivotGoal;
  private final Timer timer;

  public ShooterPivot() {
    pivot = new SparkMax(ShooterConstants.pivotID,  MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();

    //pivot.restoreFactoryDefaults();
    pivotConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);    
    pivotConfig.absoluteEncoder
      .zeroOffset(ShooterConstants.pivotZeroOffset.getRadians())
      .positionConversionFactor(ShooterConstants.rotationsToRadians)
      .velocityConversionFactor(ShooterConstants.rotationsToRadians);
    pivotConfig.closedLoop
      .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    pivotConfig.smartCurrentLimit(40);
    
    thruBore = pivot.getAbsoluteEncoder();

    // pivotPID = pivot.getClosedLoopController();
    // pivotPID.setP(ShooterConstants.kP);
    // pivotPID.setI(ShooterConstants.kI);
    // pivotPID.setD(ShooterConstants.kD);
    // pivotPID.setFeedbackDevice(thruBore);

    pivotBottomLimit = new DigitalInput(ShooterConstants.pivotBottomLimitPort);

    pivotProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        ShooterConstants.maxSpeed, 
        ShooterConstants.maxAccel));
    
    pivotGoal = ShooterConstants.pivotLowerLimit;
    pivotStart = new TrapezoidProfile.State(ShooterConstants.pivotLowerLimit.getRadians(), 0);
    lastPivotVelocitySetpoint = 0;

    pivotFF = new ArmFeedforward(
      ShooterConstants.kS, 
      ShooterConstants.kG, 
      ShooterConstants.kV);
    
      pivotFFValue = 0;

    // pivot.burnFlash();
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    timer = new Timer();
    //timer.start();
    
    pivotCharacterizer = new SysIdRoutine(new SysIdRoutine.Config(), pivotmechanism);
  }

  public void voltageDrive (Voltage volts) {
    pivot.setVoltage(volts.in(Volts));
  }

  public void log(SysIdRoutineLog log) {
    double voltage = pivot.getAppliedOutput() * pivot.getBusVoltage();
    double angularPos = thruBore.getPosition();
    double angularVel = thruBore.getVelocity();

    log.motor("shooterpivot")
      .voltage(m_appliedVoltage.mut_replace(voltage, Volts))
      .angularPosition(m_angle.mut_replace(angularPos, Radians))
      .angularVelocity(m_velocity.mut_replace(angularVel, RadiansPerSecond));
  }
/*
  public double getRadiansFromRaw() {
    return thruBore.getPosition() * 2 * Math.PI - 4.9754;
  }
  public double getDegreesFromRaw() {
    return thruBore.getPosition() * 360 - 285.07;
  }
  public double rawToRadians(double raw) {
    return raw * 2 * Math.PI - 4.9754;
  }
*/
  public double rawToDegrees(double raw) {
    return raw * 360 - 285.07;
  }

  public double getDegrees() {
    return thruBore.getPosition() * ShooterConstants.radiansToDegrees;
  }

  public void setPivotAngle(Rotation2d angle) {
    if (angle.getRadians() != pivotGoal.getRadians()) {
      pivotGoal = angle;
      pivotStart = new TrapezoidProfile.State(thruBore.getPosition(), thruBore.getVelocity());
      timer.stop();
      timer.reset();
      timer.start();
    }
  }

  public void setSpeed(double speed) {
    pivot.set(speed);
  }

  public void setVoltage(double voltage) {
    pivot.setVoltage(voltage);
  }

  public void stopShooterPivot() {
    pivot.stopMotor();
  }

  public boolean getPivotLimit() {
    return pivotBottomLimit.get();
  }

  public double getThruBore() {
    return thruBore.getPosition();
  }

  public boolean getSoftUpperLimit() {
    return (thruBore.getPosition() > ShooterConstants.pivotUpperLimit.getRadians());
    //return (getDegreesFromRaw() > ShooterConstants.pivotUpperLimit.getDegrees());
  }

  public boolean getSoftLowerLimit() {
    return (thruBore.getPosition()< ShooterConstants.pivotLowerLimit.getRadians());
  }

  public void setBrakeMode() {
    pivotConfig.idleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
/*
    if (pivotGoal.getRadians() >= ShooterConstants.pivotUpperLimit.getRadians()) {
      pivotGoal = ShooterConstants.pivotUpperLimit;
    }

    pivotSetpoint = pivotProfile.calculate(
      timer.get(), 
      pivotStart, 
      new TrapezoidProfile.State(pivotGoal.getRadians(), 0));
    
    pivotFFValue = pivotFF.calculate(pivotSetpoint.position, pivotSetpoint.velocity);
    //pivotPID.setReference(pivotSetpoint.position, ControlType.kPosition, 0, pivotFFValue, ArbFFUnits.kVoltage);
*/
    SmartDashboard.putNumber("Pivot/Pivot Raw", thruBore.getPosition());
    SmartDashboard.putNumber("Pivot/Pivot Degrees", getDegrees());
    SmartDashboard.putNumber("Pivot/Velocity", thruBore.getVelocity());
    SmartDashboard.putNumber("Pivot/Volts", pivot.getAppliedOutput() * pivot.getBusVoltage());

    SmartDashboard.putBoolean("Pivot/Upper Pivot Soft", getSoftUpperLimit());
    SmartDashboard.putBoolean("Pivot/Lower Pivot Soft", getSoftLowerLimit());
  }

  public Command Up(double speed) {
      return this.startEnd(() -> this.setSpeed(speed), () -> this.stopShooterPivot());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setSpeed(-speed), () -> this.stopShooterPivot());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotCharacterizer.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotCharacterizer.dynamic(direction);
  }

}
