// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightIntake;

public class CentretoNote extends Command {
  Drivetrain drivetrain;
  LimelightIntake limelightIntake;
  private final PIDController pid;
  public CentretoNote(Drivetrain d, LimelightIntake li) {
    this.drivetrain = d;
    this.limelightIntake = li;
    this.pid = new PIDController(0.01, 0.01, 0); //0.032, 0.018
    addRequirements(drivetrain, limelightIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("CentreToNote Start");
    pid.reset();
    pid.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed;
    double rightSpeed;
    boolean seesNote;
    if (limelightIntake.getDoubleEntry("tv") == 1.0) {
      seesNote = true;
      leftSpeed = -pid.calculate(limelightIntake.getX(), 0);
      rightSpeed = pid.calculate(limelightIntake.getX(), 0);
    } else {
      leftSpeed = 0;
      rightSpeed = 0;
      seesNote = false;
    }
    SmartDashboard.putNumber("Note Error", pid.getPositionError());
    SmartDashboard.putBoolean("Sees Note", seesNote);

    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
    System.out.println("CentreToNote End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
