// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveStraightDistance extends Command {
  private final Drivetrain drivetrain;
  private final PIDController pid;
  private final double speed;
  private final double distanceToDrive;
  
  public DriveStraightDistance(Drivetrain drivetrain, double speed, double distance) {
    this.drivetrain = drivetrain;
    this.pid = new PIDController(0.02, 0, 0);
    this.speed = speed;
    this.distanceToDrive = distance + drivetrain.getDistance();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveStraight Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = pid.calculate(drivetrain.getAngle(), 0);
    double rightSpeed = -pid.calculate(drivetrain.getAngle(),0);

    drivetrain.tankDrive(speed + leftSpeed, speed + rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0, 0);
    drivetrain.stopMotors();
    System.out.println("DriveStraight End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getDistance() > distanceToDrive;
  }
}
