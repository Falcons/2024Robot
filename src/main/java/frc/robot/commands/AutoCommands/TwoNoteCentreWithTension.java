// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommands.CentretoNote;
import frc.robot.commands.DriveCommands.DriveStraight;
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
public class TwoNoteCentreWithTension extends SequentialCommandGroup {
  /** Creates a new TwoNoteCentreWithTension. */
  public TwoNoteCentreWithTension(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterPivot shooterpivot, LimelightShooter ls, LimelightIntake li) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new OneNoteWithTension(drivetrain, intake, shooter, shooterpivot, ls),

      new SetShooterTwoPID(shooterpivot, 36.4).until(() -> shooterpivot.getDegreesFromRaw() < 38),

      //drive and pickup note in front
      new CentretoNote(drivetrain, li),
      new ParallelCommandGroup(
        new Extend(intake),
        new DriveStraight(drivetrain, 0.45),
        new SetShooterTwoPID(shooterpivot, 36.4)
      ).until(intake::hasNote),
      new Retract(intake),

      //drive back to shooter
      new ParallelRaceGroup(
        drivetrain.ramsete(drivetrain.generateTrajectory(List.of(), new Pose2d(6, 0, new Rotation2d(0)))),
        new SetShooterPosition(shooterpivot, ls)//SetShooterTwoPID(shooterpivot, shooterpivot.rawToDegrees(0.909))
      ),
      new ParallelCommandGroup(
        new SetShooterPosition(shooterpivot, ls),
        new Shoot(shooter, 1, 0.95).withTimeout(3),
        new SequentialCommandGroup(
          new WaitCommand(2), 
          new EjectNote(intake, 1).withTimeout(2))
          )
    );
  }
}
