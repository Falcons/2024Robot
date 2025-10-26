// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.playingwithfusion.TimeOfFlight;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax pivot = new SparkMax(IntakeConstants.pivotID, MotorType.kBrushless);
  private final SparkMax wheels = new SparkMax(IntakeConstants.wheelID, MotorType.kBrushless);

  private final SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private final SparkMaxConfig wheelsConfig = new SparkMaxConfig();

  private final SparkAbsoluteEncoder intakeThruBore = pivot.getAbsoluteEncoder();

  //private final DigitalInput intakeLimit = new DigitalInput(IntakeConstants.intakeBottomLimit);

  private final TimeOfFlight tof = new TimeOfFlight(0);

  public Intake() {
    // pivot.restoreFactoryDefaults();
    // wheels.restoreFactoryDefaults();

    pivotConfig.smartCurrentLimit(40);
    wheelsConfig.smartCurrentLimit(30);

    // pivot.burnFlash();
    // wheels.burnFlash();
    applyConfig();
  }

  public void applyConfig() {
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wheels.configure(wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void IntakeNote(double speed){
    wheels.set(-speed);
  }

  public void EjectNote(double speed) {
    wheels.set(speed);
  }

  public void stopIntake() {
    wheels.stopMotor();
  }

  public void pivotSpeed(double speed) {
    pivot.set(speed);
  }

  public void stopIntakePivot() {
    pivot.stopMotor();
  }

  public double getIntakeAngle() {
    return intakeThruBore.getPosition();
  }

  public boolean getBottomSoftLimit() {
    return (intakeThruBore.getPosition() > IntakeConstants.intakeInAngle);
  }

  public boolean getUpperSoftLimit() {
    return (intakeThruBore.getPosition() < IntakeConstants.intakeOutAngle);
  }

  public double getTOF() {
    return tof.getRange();
  }

  public boolean hasNote() {
    return (tof.getRange() < 155);
  }

  public String getRetractedorExtended() {
    if (getIntakeAngle() > 0.365) {
      return "Retracted";
    } else {
      return "Extended";
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Intake Thru Bore", intakeThruBore.getPosition());
    SmartDashboard.putNumber("Intake/TOF", tof.getRange());
    SmartDashboard.putBoolean("Intake/Has Note", hasNote());
    //SmartDashboard.putString("Intake", getRetractedorExtended());
  }

  public Command IntakeNoteCmd(double speed) {
    return this.startEnd(() -> this.IntakeNote(speed), () -> this.stopIntake());
  }

  public Command EjectNoteCmd(double speed) {
    return this.startEnd(() -> this.EjectNote(speed), () -> this.stopIntake());
  }

  public Command Extend(double speed) {
    return this.startEnd(() -> this.pivotSpeed(-speed), () -> this.stopIntakePivot());
  }

  public Command Retract(double speed) {
    return this.startEnd(() -> this.pivotSpeed(speed), () -> this.stopIntakePivot());
  }
}
