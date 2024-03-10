// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class SetShooterFixed extends Command {
  private final ShooterPivot shooterpivot;
  private final double position;
  private final PIDController pid;
  public SetShooterFixed(ShooterPivot shooterpivot, double pos) {
    this.shooterpivot = shooterpivot;
    this.position = pos;
    this.pid = new PIDController(1.7, 0.04, 0);
    addRequirements(shooterpivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Fixed Start");
    pid.reset();
    shooterpivot.setBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = (pid.calculate(shooterpivot.getThruBore(), position));
    shooterpivot.setSpeed(-speed);
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
