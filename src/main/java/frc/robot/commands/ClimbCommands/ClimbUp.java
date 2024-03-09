// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbUp extends Command {
  private final Climb climb;
  private final Supplier<Double> leftSpeed, rightSpeed;
  public ClimbUp(Climb c, Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {
    this.climb = c;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.setBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeLeftSpeed = leftSpeed.get();
    double realTimeRightSpeed = rightSpeed.get();
    if (realTimeLeftSpeed < 0.1 && realTimeLeftSpeed > -0.1 )  {
      realTimeLeftSpeed = 0;
    }
    if (realTimeRightSpeed < 0.1 && realTimeRightSpeed > -0.1) {
      realTimeRightSpeed = 0;
    }

    if (climb.getLeftClimbLimit()) {
      if (realTimeLeftSpeed >= 0) {
        realTimeLeftSpeed = 0;
      }
    }

    if (climb.getRightClimbLimit()) {
      if (realTimeRightSpeed >= 0) {
        realTimeRightSpeed = 0;
      }
    }

    climb.setLeftClimb(realTimeLeftSpeed);
    climb.setRightClimb(realTimeRightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
