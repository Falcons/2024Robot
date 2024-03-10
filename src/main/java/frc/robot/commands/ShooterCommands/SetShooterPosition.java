// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.ShooterPivot;

public class SetShooterPosition extends Command {
  private final ShooterPivot shooterpivot;
  private final LimelightShooter limelightshooter;
  private double pos;
  private final PIDController pid;
  //private final ArmFeedforward armFF;
  
  public SetShooterPosition(ShooterPivot shooterpivot, LimelightShooter ls) {
    this.shooterpivot = shooterpivot;
    this.limelightshooter = ls;
    this.pos = shooterpivot.returnClosest(Units.inchesToMeters(limelightshooter.getDistanceSpeaker()) - ShooterConstants.speakerDepth);
    this.pid = new PIDController(1, 0, 0);
    addRequirements(shooterpivot, limelightshooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(pos);
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pos = shooterpivot.returnClosest(Units.inchesToMeters(limelightshooter.getDistanceSpeaker()) - ShooterConstants.speakerDepth);
    double angle = shooterpivot.getHashValue(pos);

    double speed = (pid.calculate(shooterpivot.getThruBore(), angle));
    shooterpivot.setSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterpivot.getShooterMap();
    shooterpivot.stopShooterPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterpivot.getSoftUpperLimit();
  }
}
