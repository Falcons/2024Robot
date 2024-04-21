// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class SetShooterTrapMotion extends Command {
  private final ShooterPivot shooterpivot;
  private final Rotation2d giveAngle;

  public SetShooterTrapMotion(ShooterPivot shooterpivot, double angle) {
    this.shooterpivot = shooterpivot;
    this.giveAngle = new Rotation2d(angle);

    addRequirements(shooterpivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Fixed Start");
    shooterpivot.setBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterpivot.setPivotAngle(giveAngle);
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
