// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class Up extends Command {
  private final ShooterPivot shooterpivot;
  private final double speed;

  public Up(ShooterPivot shooterpivot, double speed) {
    this.shooterpivot = shooterpivot;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterpivot.setSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterpivot.setSpeed(0.05);
    shooterpivot.stopShooterPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooterpivot.getSoftUpperLimit());
  }
}
