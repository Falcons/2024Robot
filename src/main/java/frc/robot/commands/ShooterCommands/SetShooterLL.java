// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.ShooterPivot;

public class SetShooterLL extends Command {
  private final ShooterPivot shooterpivot;
  private final LimelightShooter limelightshooter;
  private final PIDController pid;
  private final ArmFeedforward armFF;
  
  public SetShooterLL(ShooterPivot shooterpivot, LimelightShooter ls) {
    this.shooterpivot = shooterpivot;
    this.limelightshooter = ls;
    this.pid = new PIDController(0.25, 0.1, 0); //P: 0.3
    this.armFF = new ArmFeedforward(0, 0.75, 0);
    addRequirements(shooterpivot, limelightshooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;

    /* Remove after testing new auto angle
    // Calculation to convert Distance to Angle comes uses raw Pivot Encoder values,
    // conversion method changes value into usable angle
    double angle = -0.00105 * limelightshooter.getDistanceSpeaker() + 0.978;
    angle = shooterpivot.rawToDegrees(angle);
    */
    double angle = limelightshooter.getShooterPivotAutoAngle();

    if (angle < ShooterConstants.pivotLowerLimit.getDegrees()) {
      angle = ShooterConstants.pivotLowerLimit.getDegrees();
    }

    double PIDOutput = pid.calculate(shooterpivot.getPivotEncoder() * ShooterConstants.radiansToDegrees, angle);
    double FFOutput = armFF.calculate(shooterpivot.getPivotEncoder(), 0);

      if (pid.getPositionError() < 0) {
        speed = PIDOutput / 20.0;
      } else {
        speed = PIDOutput;
      }

      speed += FFOutput;

      if (shooterpivot.getSoftUpperLimit() && speed > 0 ) {
        speed = 0;
      } else if (shooterpivot.getSoftLowerLimit() && speed < 0) {
        speed = 0;
      }

    shooterpivot.setVoltage(speed);

    SmartDashboard.putBoolean("Pivot/At Angle", Math.abs(shooterpivot.getDegrees() - limelightshooter.getShooterPivotAutoAngle()) < 1);
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
