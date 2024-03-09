// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.TreeMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(ShooterConstants.pivotID,  MotorType.kBrushless);
  private final SparkAbsoluteEncoder thruBore = pivot.getAbsoluteEncoder();
  private final DigitalInput pivotBottomLimit = new DigitalInput(0);

  TreeMap<Double, Double> shooterPivotMap = new TreeMap<Double, Double>();

  public ShooterPivot() {
    //thruBore.setZeroOffset(ShooterConstants.thruBoreZeroOffset);
    thruBore.setPositionConversionFactor(1);
    setShooterPivotMap();
  }
  
  public void setShooterPivotMap() {
    shooterPivotMap.put(32.75, 0.95);
    shooterPivotMap.put(36.17, 0.945);
    shooterPivotMap.put(40.42, 0.933);
    shooterPivotMap.put(45.65, 0.927);
    shooterPivotMap.put(50.22, 0.917);
    shooterPivotMap.put(55.4, 0.923);
    shooterPivotMap.put(58.76, 0.908);
    shooterPivotMap.put(62.1, 0.909);
    shooterPivotMap.put(65.59, 0.909);
    shooterPivotMap.put(70.0, 0.906);
    shooterPivotMap.put(74.54, 0.898);
    shooterPivotMap.put(78.12, 0.899);
    shooterPivotMap.put(83.37, 0.894);
    shooterPivotMap.put(87.43, 0.894);
    shooterPivotMap.put(89.8, 0.894);
  }

  public double getHashValue(double key) {
    return shooterPivotMap.get(key);
  }

  public double findDistance (double pos){
    double prevdiff = 0;
    double prevpos = 0;
    int defaultAngle = 0;
    int hashLen = shooterPivotMap.size();
    int selector = 0;


    for (double distance : getShooterMap().keySet()) {
      double diff = pos - distance;

      if (prevdiff < diff) {
        selector = 1;
      }
      else if(defaultAngle < hashLen){
        selector = 2;
      }
      else {
      prevdiff = diff;
      prevpos = pos;
      }
      defaultAngle++;
    }
    if (selector == 1){
      return prevpos;
    }
    else return pos;
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
    return (thruBore.getPosition() > 0.953);
  }

  public boolean getSoftLowerLimit() {
    return (thruBore.getPosition() < 0.88);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Encoder", thruBore.getPosition());
    SmartDashboard.putNumber("Pivot Speed", pivot.get());

    SmartDashboard.putBoolean("Pivot Limit Switch", getPivotLimit());
    SmartDashboard.putBoolean("Soft Limit Enabled", getSoftUpperLimit());
  }

  public Command Up(double speed) {
      return this.startEnd(() -> this.setSpeed(-speed), () -> this.stopShooterPivot());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setSpeed(speed), () -> this.stopShooterPivot());
  }
}
