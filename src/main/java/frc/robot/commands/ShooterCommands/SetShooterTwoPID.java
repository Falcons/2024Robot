// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterPivot;

public class SetShooterTwoPID extends Command {
  private final ShooterPivot shooterpivot;
  private final double position;
  private final PIDController pidToSetpoint;
  private final ArmFeedforward armFF;
  public SetShooterTwoPID(ShooterPivot shooterpivot, double pos) {
    this.shooterpivot = shooterpivot;
    this.position = pos;
    this.pidToSetpoint = new PIDController(0.2, 0, 0); //0.3, 0, 0
    this.armFF = new ArmFeedforward(0, 0.75, 0);
    addRequirements(shooterpivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Fixed Start");
    shooterpivot.setBrakeMode();
    pidToSetpoint.reset();
    pidToSetpoint.setSetpoint(position);
    pidToSetpoint.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double FFOutput = armFF.calculate(shooterpivot.getThruBore(), 0);
    double PIDToSetpointOutput = pidToSetpoint.calculate(shooterpivot.getThruBore() * ShooterConstants.radiansToDegrees, position);
    
    double speed;

    if (pidToSetpoint.getPositionError() < 0) {
      speed = PIDToSetpointOutput / 5.0;
    } else {
       speed = PIDToSetpointOutput;
    }
    if (pidToSetpoint.atSetpoint()) {
      speed += FFOutput;
    }

    if (shooterpivot.getSoftUpperLimit() && speed > 0 ) {
      speed = 0;
    } else if (shooterpivot.getSoftLowerLimit() && speed < 0) {
      speed = 0;
    }

    shooterpivot.setVoltage(speed);

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
    return false;
  }
}
