// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterPivot;

public class SetShooterTwoPID extends Command {
  private final ShooterPivot shooterpivot;
  private final double position;
  private final PIDController pidToSetpoint;
  private final PIDController pidFixed;
  private final ArmFeedforward armFF;
  public SetShooterTwoPID(ShooterPivot shooterpivot, double pos) {
    this.shooterpivot = shooterpivot;
    this.position = pos;
    this.pidToSetpoint = new PIDController(0.2, 0, 0);

    this.pidFixed = new PIDController(0.4, 0.01, 0);
    this.armFF = new ArmFeedforward(0, 0.35, 1.95, 0.02);
    addRequirements(shooterpivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Fixed Start");
    shooterpivot.setBrakeMode();
    pidToSetpoint.reset();
    pidFixed.reset();
    pidToSetpoint.setSetpoint(position);
    pidToSetpoint.setTolerance(1);
    pidFixed.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double FFOutput = armFF.calculate(position * ShooterConstants.degreesToRadians, 0);
    double PIDToSetpointOutput = pidToSetpoint.calculate(shooterpivot.getDegrees(), position);
    double PIDFixedOutput = pidFixed.calculate(shooterpivot.getDegrees(), position);
    double speed;

    if (pidToSetpoint.atSetpoint()) {
      speed = PIDFixedOutput;
    } else {
      speed = PIDToSetpointOutput;
    }

    if (shooterpivot.getSoftUpperLimit() && speed > 0 ) {
      speed = 0;
    } else if (shooterpivot.getSoftLowerLimit() && speed < 0) {
      speed = 0;
    }

    shooterpivot.setVoltage(-speed);
    SmartDashboard.putBoolean("At Setpoint", pidToSetpoint.atSetpoint());
    SmartDashboard.putNumber("Error", pidToSetpoint.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Fixed End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterpivot.getSoftUpperLimit();
  }
}
