// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private DigitalOutput white = new DigitalOutput(4);
  private DigitalOutput green = new DigitalOutput(5);
  private DigitalOutput yellow = new DigitalOutput(6);
  private DigitalOutput blue = new DigitalOutput(7);
  public LEDs() {}

  public void setWhite() {
    white.set(true);
    green.set(false);
    yellow.set(false);
    blue.set(false);
  }
  public void setGreen() {
    white.set(false);
    green.set(true);
    yellow.set(false);
    blue.set(false);
  }
  public void setYellow() {
    white.set(false);
    green.set(false);
    yellow.set(true);
    blue.set(false);
  }
  public void setBlue() {
    white.set(false);
    green.set(false);
    yellow.set(false);
    //blue.set(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("LED/White", white.get());
    SmartDashboard.putBoolean("LED/Green", green.get());
    SmartDashboard.putBoolean("LED/Yellow", yellow.get());
    SmartDashboard.putBoolean("LED/Blue", blue.get());
  }
}
