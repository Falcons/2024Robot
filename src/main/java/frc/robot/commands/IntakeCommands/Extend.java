// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class Extend extends Command {
  private final Intake intake;
  private final PIDController pid;
  private final PIDController pidFixed;
  public Extend(Intake intake) {
    this.intake = intake;
    this.pid = new PIDController(1.5, 0, 0);
    this.pidFixed = new PIDController(1.6, 0, 0);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Extend Start");
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pid.calculate(intake.getIntakeAngle(), IntakeConstants.intakeOutAngle);
    

    if (pid.atSetpoint()) {
      intake.IntakeNote(0.3);
      //speed = pidFixed.calculate(intake.getIntakeAngle(), IntakeConstants.intakeOutAngle);
    }

    intake.pivotSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Extend End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}