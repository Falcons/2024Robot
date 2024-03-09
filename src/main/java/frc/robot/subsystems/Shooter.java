// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX leftFlywheel = new TalonFX(ShooterConstants.leftFlywheelID);
  private final TalonFX rightFlywheel = new TalonFX(ShooterConstants.rightFlywheelID);

  public Shooter() {
    rightFlywheel.setInverted(true);
  }

  public void fire(double leftSpeed, double rightSpeed) {
    leftFlywheel.set(leftSpeed);
    rightFlywheel.set(rightSpeed);
  }

  public void stopShooter() {
    leftFlywheel.stopMotor();
    rightFlywheel.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command Shoot(double leftSpeed, double rightSpeed) {
    return this.startEnd(() -> this.fire(leftSpeed, rightSpeed), () -> this.stopShooter());
  }

  public Command setSpeed(double leftSpeed, double rightSpeed) {
    return this.runOnce(() -> fire(leftSpeed, rightSpeed));
  }

  public Command stop() {
    return this.runOnce(() -> stopShooter());
  }
}
