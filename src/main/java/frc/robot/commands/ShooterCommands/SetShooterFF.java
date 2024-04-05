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

public class SetShooterFF extends Command {
  private final ShooterPivot shooterpivot;
  private final ArmFeedforward armFF;
  public SetShooterFF(ShooterPivot shooterpivot) {
    this.shooterpivot = shooterpivot;
    this.armFF = new ArmFeedforward(0, 0.75, 0);
    addRequirements(shooterpivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("FF Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double FFOutput = armFF.calculate(shooterpivot.getThruBore(), 0);

    shooterpivot.setVoltage(FFOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterpivot.stopShooterPivot();
    System.out.println("FF End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
