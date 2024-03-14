// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class RotateToAngle extends Command {
  private final Drivetrain drivetrain;
  private final double angle;
  private final PIDController pid;
  public RotateToAngle(Drivetrain drivetrain, double angle) {
    this.drivetrain = drivetrain;
    this.angle = angle;
    this.pid = new PIDController(0.02, 0, 0);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RotateToAngle Start");
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = pid.calculate(drivetrain.getAngle(), angle);
    double rightSpeed = -pid.calculate(drivetrain.getAngle(),angle);

    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("RotateToAngle End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
