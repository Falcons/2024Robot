// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommands.CentretoNote;
import frc.robot.commands.DriveCommands.CentretoSpeaker;
import frc.robot.commands.DriveCommands.DriveStraight;
import frc.robot.commands.DriveCommands.RotateToAngle;
import frc.robot.commands.IntakeCommands.EjectNote;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.IntakeNote;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.ShooterCommands.SetShooterPosition;
import frc.robot.commands.ShooterCommands.SetShooterTwoPID;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueAmpSideTwoNote extends SequentialCommandGroup {
  /** Creates a new SideTaxi. */
  public BlueAmpSideTwoNote(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterPivot shooterpivot, LimelightShooter ls, LimelightIntake li) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new SetShooterTwoPID(shooterpivot, 56.93).until(() -> shooterpivot.getDegreesFromRaw() > 56),
        new ParallelCommandGroup(
          new Shoot(shooter, 1, 0.95).withTimeout(3),
          new SequentialCommandGroup(
            new WaitCommand(2), 
            new EjectNote(intake, 1).withTimeout(1))
            )
    ),
      //new SetShooterTwoPID(shooterpivot, 36.4).until(() -> shooterpivot.getDegreesFromRaw() < 41),
      new DriveStraight(drivetrain, 0.5).until(() -> drivetrain.getDistance() > 0.25),
      new ParallelCommandGroup(
        new SetShooterTwoPID(shooterpivot, 36.4).until(() -> shooterpivot.getDegreesFromRaw() < 43).withTimeout(0.75),
        new RotateToAngle(drivetrain, 55).withTimeout(0.75)
      ),

      //centre and pickup
      new CentretoNote(drivetrain, li),
      new Extend(intake),
      new ParallelCommandGroup(
        new DriveStraight(drivetrain, 0.5),
        new IntakeNote(intake, IntakeConstants.intakeSpeed)
      ).until(intake::hasNote),
      new Retract(intake),
      
      //centre with speaker
      new RotateToAngle(drivetrain, -30).withTimeout(1),
      new CentretoSpeaker(drivetrain, ls),

      new DriveStraight(drivetrain, -0.5).until(() -> drivetrain.getDistance() < -0.6),
      //shoot
      new ParallelCommandGroup(
        new SetShooterPosition(shooterpivot, ls),
        new ParallelCommandGroup(
          new Shoot(shooter, 1, 0.95).withTimeout(3),
          new SequentialCommandGroup(
            new WaitCommand(1.5), 
            new EjectNote(intake, 1).withTimeout(1.5)
          )
        )
      )

    );
  }
}
