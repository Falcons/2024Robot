// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(IntakeConstants.pivotID, MotorType.kBrushless);
  private final CANSparkMax wheels = new CANSparkMax(IntakeConstants.wheelID, MotorType.kBrushless);

  private final SparkAbsoluteEncoder intakeThruBore = pivot.getAbsoluteEncoder();

  //private final DigitalInput intakeLimit = new DigitalInput(IntakeConstants.intakeBottomLimitDIOPort);

  private final TimeOfFlight tof = new TimeOfFlight(0);

  public Intake() {
    pivot.restoreFactoryDefaults();
    wheels.restoreFactoryDefaults();

    pivot.setSmartCurrentLimit(40);
    wheels.setSmartCurrentLimit(30);

    pivot.burnFlash();
    wheels.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Intake Thru Bore", intakeThruBore.getPosition());
    SmartDashboard.putNumber("Intake/TOF", tof.getRange());
    SmartDashboard.putBoolean("Intake/Has Note", hasNote());
    //SmartDashboard.putString("Intake", getRetractedorExtended());
  }

  // Wheel Methods
  public void IntakeNote(double speed){
    wheels.set(-speed);
  }

  public void EjectNote(double speed) {
    wheels.set(speed);
  }

  public void stopIntake() {
    wheels.stopMotor();
  }

  //Intake Pivot Methods
  public void pivotSpeed(double speed) {
    pivot.set(speed);
  }

  public void stopIntakePivot() {
    pivot.stopMotor();
  }

  public String getRetractedorExtended() {
    if (getIntakeAngle() > 0.365) {
      return "Retracted";
    } else {
      return "Extended";
    }
  }

  /**
   * @return raw angle in rotations
   */
  public double getIntakeAngle() {
    return intakeThruBore.getPosition();
  }

  //Limits
  public boolean getSoftUpperLimit() {
    return (intakeThruBore.getPosition() < IntakeConstants.intakeOutAngle);
  }

  public boolean getSoftBottomLimit() {
    return (intakeThruBore.getPosition() > IntakeConstants.intakeInAngle);
  }

  //TOF distance sensor
  public double getTOF() {
    return tof.getRange();
  }

  public boolean hasNote() {
    return (tof.getRange() < 155);
  }

  //Wheel Commands
  public Command IntakeNoteCmd(double speed) {
    return this.startEnd(() -> this.IntakeNote(speed), () -> this.stopIntake());
  }

  public Command EjectNoteCmd(double speed) {
    return this.startEnd(() -> this.EjectNote(speed), () -> this.stopIntake());
  }
}
