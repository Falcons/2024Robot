// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommands.DriveStraight;
import frc.robot.commands.DriveCommands.DriveToTarget;
import frc.robot.commands.DriveCommands.MoveToShooter;
import frc.robot.commands.IntakeCommands.EjectNote;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.IntakeNote;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.ShooterCommands.SetShooterFixed;
import frc.robot.commands.ShooterCommands.SetShooterPosition;
import frc.robot.commands.ShooterCommands.SetShooterTwoPID;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAutoTest extends SequentialCommandGroup {
  /** Creates a new TwoNoteAuto. */
  public TwoNoteAutoTest(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterPivot shooterpivot, LimelightShooter ls, LimelightIntake li) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //initial shooting
      new ParallelCommandGroup(
        new SetShooterTwoPID(shooterpivot, 56.93).until(() -> shooterpivot.getDegreesFromRaw() > 56),
        new ParallelCommandGroup(
          new Shoot(shooter, 1, 0.95).withTimeout(3),
          new SequentialCommandGroup(
            new WaitCommand(2), 
            new EjectNote(intake, 1).withTimeout(1))
        )
      ),
      new SetShooterTwoPID(shooterpivot, 36.4).until(() -> shooterpivot.getDegreesFromRaw() < 38),

      //drive and pickup note in front
      new Extend(intake),
      new ParallelCommandGroup(
        //new DriveStraight(drivetrain, 0.45),
        new InstantCommand(() -> drivetrain.tankDrive(1,-1)).until(() -> li.getTV() == true),
        new DriveToTarget(drivetrain, li),
        new IntakeNote(intake, IntakeConstants.intakeSpeed),
        new SetShooterTwoPID(shooterpivot, 36.4)
      ).until(intake::hasNote),
      new Retract(intake),

      //drive back turn on shooter
      new ParallelCommandGroup(
        //new DriveStraight(drivetrain, -0.45),
        new MoveToShooter(drivetrain, ls.getBotpose(), ls),
        new SetShooterPosition(shooterpivot, ls)//SetShooterTwoPID(shooterpivot, shooterpivot.rawToDegrees(0.909))
      ).until(() -> drivetrain.getDistance() <= 0),
      new ParallelCommandGroup(
        new Shoot(shooter, 1, 0.95).withTimeout(3),
        new SequentialCommandGroup(
          new WaitCommand(2), 
          new EjectNote(intake, 1).withTimeout(2))
          )
    
    );
  }
}
