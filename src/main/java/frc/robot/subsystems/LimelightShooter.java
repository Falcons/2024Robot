// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightShooter extends SubsystemBase {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-shooter");
  public LimelightShooter() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LLShooter/Distance Speaker", getDistanceSpeaker());
    SmartDashboard.putNumber("LLShooter/Setpoint Angle", getShooterPivotAutoAngle());
  }

  public void setPriorityID() {
    if (DriverStation.getAlliance().isEmpty()) return;
    
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      setDoubleEntry("priorityid", 4);
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      setDoubleEntry("priorityid", 7);
    }
  }

  public double getShooterPivotAutoAngle() {
    return (-0.00105 * getDistanceSpeaker() + 0.978) * 360 - 285.07;
  }

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

  public double[] getBotpose() {
    return getArrayEntry("botpose_wpiblue");
  }

  public double getDistanceSpeaker() {
    double targetOffsetAngle_Vertical = getDoubleEntry("ty");

    double limelightMountAngleDegrees = 32;

    double limelightLensHeightInches = 15;

    double goalHeightInches = 51.875;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.1415 / 180.0);

    double distanceFromLimelightGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    return distanceFromLimelightGoal;

  }
}
