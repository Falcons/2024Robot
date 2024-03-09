// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends Command {
  private final Drivetrain drivetrain;
  private final PIDController pid;
  private final Timer timer;

  private final double speed;
  private final double time;
  
  public DriveStraight(Drivetrain drivetrain, double speed, double time) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.pid = new PIDController(0.04, 0.05, 0);
    this.time = time;
    this.timer = new Timer();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("DriveStraight Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Timer", timer.get());
    double leftSpeed = -pid.calculate(drivetrain.getAngle(), 0);
    double rightSpeed = pid.calculate(drivetrain.getAngle(),0);

    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0, 0);
    timer.stop();
    System.out.println("DriveStraight end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
