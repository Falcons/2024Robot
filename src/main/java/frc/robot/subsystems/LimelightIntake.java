// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightIntake extends SubsystemBase {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-intake");
  public LimelightIntake() {}

  @Override
  public void periodic() {}

  public double getDoubleEntry(String entry) {
    return table.getEntry(entry).getDouble(0);
  }

  public double[] getArrayEntry(String entry) {
      return table.getEntry(entry).getDoubleArray(new double[6]);
  }

  public void setDoubleEntry(String entry, Number number) {
      table.getEntry(entry).setNumber(number);
  }

  public void setArrayEntry(String entry, double[] array) {
      table.getEntry(entry).setDoubleArray(array);
  }

  public double getX() {
      return getDoubleEntry("tx");
  }

  public double getY() {
      return getDoubleEntry("ty");
  }

  public double getID() {
      return getDoubleEntry("tid");
  }
}
