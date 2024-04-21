// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX leftFlywheel = new TalonFX(ShooterConstants.leftFlywheelID);
  private final TalonFX rightFlywheel = new TalonFX(ShooterConstants.rightFlywheelID);

  private double leftSpeed;
  private double rightSpeed;

  public Shooter() {
    // Setting current limits
    var leftFlywheelConfigurator = leftFlywheel.getConfigurator();
    var rightFlywheelConfigurator = rightFlywheel.getConfigurator();
    var currentlimitConfigs = new CurrentLimitsConfigs();
    
    currentlimitConfigs.SupplyCurrentLimit = 30.0;
    currentlimitConfigs.StatorCurrentLimit = 30.0;
    currentlimitConfigs.SupplyCurrentLimitEnable = true;
    currentlimitConfigs.StatorCurrentLimitEnable = true;

    leftFlywheelConfigurator.apply(currentlimitConfigs);
    rightFlywheelConfigurator.apply(currentlimitConfigs);

    rightFlywheel.setInverted(true);
  }

  @Override
  public void periodic() {
    leftSpeed = leftFlywheel.getVelocity().getValueAsDouble();
    rightSpeed = rightFlywheel.getVelocity().getValueAsDouble();
    
    SmartDashboard.putNumber("Shooter/Left Shooter Speed", leftSpeed);
    SmartDashboard.putNumber("Shooter/Right Shooter Speed", rightSpeed);
    SmartDashboard.putBoolean("Shooter/Shoot", leftSpeed > 90 && rightSpeed > 90);
  }

  //Shooter Methods
  public void fire(double leftSpeed, double rightSpeed) {
    leftFlywheel.set(leftSpeed);
    rightFlywheel.set(rightSpeed);
  }

  public void stopShooter() {
    leftFlywheel.stopMotor();
    rightFlywheel.stopMotor();
  }

  /**
   * @return True when flywheels are at 90%
   */
  public boolean getShoot() {
    return leftSpeed > 90 && rightSpeed > 90;
  }

  // Shooter Commands
  public Command Shoot(double leftSpeed, double rightSpeed) {
    return this.startEnd(() -> this.fire(leftSpeed, rightSpeed), () -> this.stopShooter());
  }
}
