// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightShooter;

public class CentretoSpeaker extends Command {
  Drivetrain drivetrain;
  LimelightShooter limelightShooter;
  private final PIDController pid;
  public CentretoSpeaker(Drivetrain d, LimelightShooter ls) {
    this.drivetrain = d;
    this.limelightShooter = ls;
    this.pid = new PIDController(0.04, 0.01, 0);
    addRequirements(drivetrain, limelightShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("CentreToSpeaker Start");
    //limelightShooter.setDoubleEntry("priorityid", 7);
    //System.out.println(limelightShooter.getDoubleEntry("priorityid"));
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = pid.calculate(limelightShooter.getX(), 0);
    double rightSpeed = -pid.calculate(limelightShooter.getX(), 0);

    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
    System.out.println("CentreToSpeaker End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
