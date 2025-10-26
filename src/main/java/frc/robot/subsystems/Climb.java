// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private final SparkMax leftClimb = new SparkMax(ClimbConstants.leftClimbID, MotorType.kBrushless);
  private final SparkMax rightClimb = new SparkMax(ClimbConstants.rightClimbID, MotorType.kBrushless);

  private final SparkMaxConfig leftClimbConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightClimbConfig = new SparkMaxConfig();

  // private final RelativeEncoder leftLiftEncoder = leftClimb.getEncoder();
  // private final RelativeEncoder rightLiftEncoder = rightClimb.getEncoder();

  private final DigitalInput leftClimbLimit = new DigitalInput(ClimbConstants.leftClimbLimitPort);
  private final DigitalInput rightClimbLimit = new DigitalInput(ClimbConstants.rightClimbLimitPort);

  public Climb() {
    // leftClimb.restoreFactoryDefaults();
    // rightClimb.restoreFactoryDefaults();

    leftClimbConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40);
      
    rightClimbConfig
      .smartCurrentLimit(40)
      .inverted(true)
      .idleMode(IdleMode.kCoast);
    applyConfig();
  }

  public void setClimb(double leftSpeed, double rightSpeed) {
    leftClimb.set(leftSpeed);
    rightClimb.set(rightSpeed);
  }

  public void setLeftClimb(double speed){
    leftClimb.set(speed);
  }
  public void setRightClimb(double speed){
    rightClimb.set(speed);
  }

  public boolean getLeftClimbLimit() {
    return leftClimbLimit.get();
  }

  public boolean getRightClimbLimit() {
    return rightClimbLimit.get();
  }

  public void stopClimb() {
    leftClimb.stopMotor();
    rightClimb.stopMotor();
  }

  public void setBrakeMode() {
    leftClimbConfig.idleMode(IdleMode.kBrake);
    rightClimbConfig.idleMode(IdleMode.kBrake);
    // applyConfig(); TODO: apply config?
  }

  public void setCoastMode() {
    leftClimbConfig.idleMode(IdleMode.kCoast);
    rightClimbConfig.idleMode(IdleMode.kCoast);
    // applyConfig();
  }
  public void applyConfig() {
    leftClimb.configure(leftClimbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightClimb.configure(rightClimbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb/Left Climb Limit", leftClimbLimit.get());
    SmartDashboard.putBoolean("Climb/Right Climb Limit", rightClimbLimit.get());
  }

  public Command Up(double speed) {
    return this.startEnd(() -> this.setClimb(speed, speed), () -> this.stopClimb());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setClimb(-speed, -speed), () -> this.stopClimb());
  }
}
