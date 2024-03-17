// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.DriveStraight;
import frc.robot.commands.DriveCommands.RotateToAngle;
import frc.robot.commands.IntakeCommands.EjectNote;
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
public class BlueAmpSideOneNote extends SequentialCommandGroup {
  /** Creates a new SideTaxi. */
  public BlueAmpSideOneNote(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterPivot shooterpivot, LimelightShooter ls) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterTwoPID(shooterpivot, 56.93).until(() -> shooterpivot.getDegreesFromRaw() > 56),
      new ParallelCommandGroup(
        new Shoot(shooter, 1, 0.95).withTimeout(3),
        new SequentialCommandGroup(
          new WaitCommand(2), 
          new EjectNote(intake, 1).withTimeout(1))
          ),
      new SetShooterTwoPID(shooterpivot, 36.4).until(() -> shooterpivot.getDegreesFromRaw() < 38),
      new DriveStraight(drivetrain, 0.5).until(() -> drivetrain.getDistance() > 0.25),
      new RotateToAngle(drivetrain, 55).withTimeout(1),
      new DriveStraight(drivetrain, 0.5).withTimeout(1.5)
    );
  }
}
