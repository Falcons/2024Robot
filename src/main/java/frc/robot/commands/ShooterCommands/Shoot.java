// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  private final Shooter shooter;
  private final Timer timer;
  private double leftspeed;
  private double rightspeed;
  private double time;
  public Shoot(Shooter shooter, double leftspeed, double rightspeed, double time) {
    this.shooter = shooter;
    this.time = time;
    this.timer = new Timer();
    this.leftspeed = leftspeed;
    this.rightspeed = rightspeed;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shoot Start");
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Timer", timer.get());
    shooter.fire(leftspeed, rightspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    timer.stop();
    System.out.println("Shoot End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
