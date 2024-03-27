// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.ShooterPivot;

public class SetShooterPosition extends Command {
  private final ShooterPivot shooterpivot;
  private final LimelightShooter limelightshooter;
  private final PIDController pid;
  //private final ArmFeedforward armFF;
  
  public SetShooterPosition(ShooterPivot shooterpivot, LimelightShooter ls) {
    this.shooterpivot = shooterpivot;
    this.limelightshooter = ls;
    //this.pos = shooterpivot.returnClosest(limelightshooter.getDistanceSpeaker());
    this.pid = new PIDController(0.25, 0.1, 0); //P: 0.3
    addRequirements(shooterpivot, limelightshooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
    limelightshooter.setDoubleEntry("priorityid", ShooterConstants.priorityid);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    double angle = -0.00105 * limelightshooter.getDistanceSpeaker() + 0.978;
    angle = shooterpivot.rawToDegrees(angle);

    double PIDOutput = pid.calculate(shooterpivot.getDegreesFromRaw(), angle);

    SmartDashboard.putNumber("Error", pid.getPositionError());

      if (pid.getPositionError() < 0) {
        speed = PIDOutput / 20.0;
      } else {
        speed = PIDOutput;
      }

      if (shooterpivot.getSoftUpperLimit() && speed > 0 ) {
        speed = 0;
        System.out.println("0");
      } else if (shooterpivot.getSoftLowerLimit() && speed < 0) {
        speed = 0;
        System.out.println("0");
      }

    //shooterpivot.returnClosest(limelightshooter.getDistanceSpeaker());
    //pos = shooterpivot.returnClosest(Units.inchesToMeters(limelightshooter.getDistanceSpeaker()) - ShooterConstants.speakerDepth);
    //double angle = shooterpivot.getHashValue(pos);

    //speed = (pid.calculate(shooterpivot.getThruBore(), angle));
    shooterpivot.setVoltage(-PIDOutput);
    System.out.println(speed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto Aim Ended");
    shooterpivot.stopShooterPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
