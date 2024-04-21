// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class SetLEDs extends Command {
  private final LEDs leds;
  private final Intake intake;
  private final Shooter shooter;

  public SetLEDs(LEDs leds, Intake intake, Shooter shooter) {
    this.leds = leds;
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("LEDs Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getShoot()) {
      leds.setPurple();
    } else if (intake.getRetractedorExtended() == "Extended" && !intake.hasNote()) {
      leds.setWhite();
    } else if (intake.hasNote()) {
      leds.setGreen();
    } else {
      leds.setBlue();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("LEDs End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
