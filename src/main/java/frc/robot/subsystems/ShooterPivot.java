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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
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

  private final SysIdRoutine pivotCharacterizer;
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private Rotation2d pivotGoal;
  private final Timer timer;

  TreeMap<Double, Double> shooterPivotMap = new TreeMap<Double, Double>();

  public ShooterPivot() {
    pivot = new CANSparkMax(ShooterConstants.pivotID,  MotorType.kBrushless);
    pivot.restoreFactoryDefaults();
    pivot.setIdleMode(IdleMode.kBrake);

    thruBore = pivot.getAbsoluteEncoder();
    thruBore.setPositionConversionFactor(ShooterConstants.rotationsToDegrees);
    thruBore.setVelocityConversionFactor(ShooterConstants.rotationsToDegrees / 60.0);
    thruBore.setZeroOffset(ShooterConstants.pivotZeroOffset.getDegrees());

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
    
    pivotGoal = Rotation2d.fromDegrees(ShooterConstants.pivotLowerLimit);
    pivotStart = new TrapezoidProfile.State(getThruBore(), 0);
    lastPivotVelocitySetpoint = 0;

    pivotFF = new ArmFeedforward(
      ShooterConstants.kS, 
      ShooterConstants.kG, 
      ShooterConstants.kV);
    
      pivotFFValue = 0;

    pivot.burnFlash();

    timer = new Timer();
    timer.start();

    pivotCharacterizer = 
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
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            pivot.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(thruBore.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(thruBore.getVelocity(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

    setShooterPivotMap();
  }
  
  public void setShooterPivotMap() {
    shooterPivotMap.put(32.75, 0.95); //0.95
    shooterPivotMap.put(36.17, 0.945); //0.945
    shooterPivotMap.put(40.42, 0.933); //0.933
    shooterPivotMap.put(45.65, 0.927); //0.927
    shooterPivotMap.put(50.22, 0.917); // 0.917
    shooterPivotMap.put(55.4, 0.923); //0.923
    shooterPivotMap.put(58.76, 0.908); //0.908
    shooterPivotMap.put(62.1, 0.909); //0.909
    shooterPivotMap.put(65.59, 0.909); //0.909
    shooterPivotMap.put(70.0, 0.906); //0.906
    shooterPivotMap.put(74.54, 0.898); //0.898
    shooterPivotMap.put(78.12, 0.899); //0.899
    shooterPivotMap.put(83.37, 0.894); //0.894
    shooterPivotMap.put(87.43, 0.894); //0.894
    shooterPivotMap.put(89.8, 0.894); //0.894
  }

  public double rawToDegrees(double raw) {
    return raw * 360 - 285.07;
  }

  public double getHashValue(double key) {
    return shooterPivotMap.get(key);
  }

  public double returnClosest(double distance) {
    Double minDiff = Double.MAX_VALUE;
    Double nearest = null;
    for (Double key : shooterPivotMap.keySet()) {
      double diff = Math.abs(distance - key);
      if (diff < minDiff) {
        nearest = key;
        minDiff = diff;
      }
    }
    return nearest;
  }

  public TreeMap<Double, Double> getShooterMap() {
    return shooterPivotMap;
  }

  public void setPivotAngle(Rotation2d angle) {
    pivotGoal = angle;
    pivotStart = new TrapezoidProfile.State(getThruBore(), getThruBore() / 60.0);
    timer.stop();
    timer.reset();
    timer.start();
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

  public double getDegrees() {
    return thruBore.getPosition() * 360 - 285.07;
  }

  public boolean getSoftUpperLimit() {
    return (thruBore.getPosition() > ShooterConstants.pivotUpperLimit);
  }

  public boolean getSoftLowerLimit() {
    return (thruBore.getPosition() < ShooterConstants.pivotLowerLimit);
  }

  public void setBrakeMode() {
    pivot.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    pivotSetpoint = pivotProfile.calculate(
      timer.get(), 
      pivotStart, 
      new TrapezoidProfile.State(pivotGoal.getRadians(), 0));
    
    pivotFFValue = pivotFF.calculate(pivotSetpoint.position, pivotSetpoint.velocity);
    //pivotPID.setReference(pivotSetpoint.position, ControlType.kPosition, 0, pivotFFValue, ArbFFUnits.kVoltage);

    SmartDashboard.putNumber("Pivot Raw", thruBore.getPosition());
    SmartDashboard.putNumber("Pivot Degrees", thruBore.getPosition() * 360 - 285.07);
    SmartDashboard.putNumber("Pivot Output", pivot.getAppliedOutput());

    SmartDashboard.putBoolean("Upper Pivot Soft", getSoftUpperLimit());
    SmartDashboard.putBoolean("Lower Pivot Soft", getSoftLowerLimit());
  }

  public Command Up(double speed) {
      return this.startEnd(() -> this.setSpeed(-speed), () -> this.stopShooterPivot());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setSpeed(speed), () -> this.stopShooterPivot());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotCharacterizer.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotCharacterizer.dynamic(direction);
  }

}
