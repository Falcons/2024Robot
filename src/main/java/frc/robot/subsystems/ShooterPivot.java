// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TreeMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ShooterPivot extends SubsystemBase {
  private final CANSparkMax pivot;
  private final SparkAbsoluteEncoder thruBore;
  private final DigitalInput pivotBottomLimit;
  

  private final TrapezoidProfile pivotProfile;
  private TrapezoidProfile.State pivotStart, pivotSetpoint;
  private final SparkPIDController pivotPID;
  private final ArmFeedforward pivotFF;

  private double pivotFFValue, lastPivotVelocitySetpoint;

  // Mutable holder for unit-safe values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private final SysIdRoutine.Mechanism pivotmechanism = new SysIdRoutine.Mechanism(this::voltageDrive, this::log, this);

  private final SysIdRoutine pivotCharacterizer;

  //private final SysIdRoutine oldpivotCharacterizer;

  private Rotation2d pivotGoal;
  private final Timer timer;

  TreeMap<Double, Double> shooterPivotMap = new TreeMap<Double, Double>();

  public ShooterPivot() {
    pivot = new CANSparkMax(ShooterConstants.pivotID,  MotorType.kBrushless);
    pivot.restoreFactoryDefaults();
    pivot.setInverted(true);
    pivot.setIdleMode(IdleMode.kBrake);
    pivot.setSmartCurrentLimit(40);

    thruBore = pivot.getAbsoluteEncoder();
    //thruBore.setPositionConversionFactor(ShooterConstants.rotationsToRadians);
    //thruBore.setVelocityConversionFactor(ShooterConstants.rotationsToRadians);
    //thruBore.setVelocityConversionFactor(ShooterConstants.rotationsToRadians / 60.0);
    //thruBore.setZeroOffset(ShooterConstants.pivotZeroOffset.getRadians());

    pivotPID = pivot.getPIDController();
    pivotPID.setP(ShooterConstants.kP);
    pivotPID.setI(ShooterConstants.kI);
    pivotPID.setD(ShooterConstants.kD);
    pivotPID.setFeedbackDevice(thruBore);

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

    pivot.burnFlash();

    timer = new Timer();
    //timer.start();
    
    pivotCharacterizer = new SysIdRoutine(new SysIdRoutine.Config(), pivotmechanism);
/*
    oldpivotCharacterizer = 
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                pivot.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(m_appliedVoltage.mut_replace(
                            pivot.getAppliedOutput() * pivot.getBusVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(getRadiansFromRaw(), Radians))
                    .angularVelocity(
                        m_velocity.mut_replace(thruBore.getVelocity(), RadiansPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  */

  }

  public void voltageDrive (Measure<Voltage> volts) {
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

  public double getRadiansFromRaw() {
    return thruBore.getPosition() * 2 * Math.PI - 4.9754;
  }
  public double getDegreesFromRaw() {
    return thruBore.getPosition() * 360 - 285.07;
  }
  public double rawToDegrees(double raw) {
    return raw * 360 - 285.07;
  }
  public double rawToRadians(double raw) {
    return raw * 2 * Math.PI - 4.9754;
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
    return (getDegreesFromRaw() > ShooterConstants.pivotUpperLimit.getDegrees());
  }

  public boolean getSoftLowerLimit() {
    return (getDegreesFromRaw() < ShooterConstants.pivotLowerLimit.getDegrees());
  }

  public void setBrakeMode() {
    pivot.setIdleMode(IdleMode.kBrake);
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
    SmartDashboard.putNumber("Pivot/Pivot Degrees", getDegreesFromRaw());
    SmartDashboard.putNumber("Pivot/Pivot Radians", getRadiansFromRaw());

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
