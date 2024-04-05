// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDriveCmd extends Command {
  private final Drivetrain drivetrain;
  private final Supplier<Double> speed, turn;
  public ArcadeDriveCmd(Drivetrain m_drivetrain, Supplier<Double> speed, Supplier<Double> turn) {
    this.drivetrain = m_drivetrain;
    this.speed = speed;
    this.turn = turn;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setSafetyEnabled(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeSpeed = speed.get();
    double realTimeTurn = turn.get();

    drivetrain.arcadeDrive(realTimeSpeed, realTimeTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
