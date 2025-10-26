// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.DriveStraight;
import frc.robot.commands.DriveCommands.RotateAngle;
import frc.robot.commands.ShooterCommands.SetShooterTwoPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedAmpBlueSourceOneNoteTaxi extends SequentialCommandGroup {
  /** Creates a new SideTaxi. */
  public RedAmpBlueSourceOneNoteTaxi(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterPivot shooterpivot, LimelightShooter ls) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new OneNoteWithTension(drivetrain, intake, shooter, shooterpivot, ls),
      new DriveStraight(drivetrain, 0.5).until(() -> drivetrain.getDistance() > 0.5),
      new ParallelRaceGroup(
        new SetShooterTwoPID(shooterpivot, 36.4).until(() -> shooterpivot.getDegrees() < 43).withTimeout(0.75),
        new RotateAngle(drivetrain, -55).withTimeout(0.75)
      ),
      new DriveStraight(drivetrain, 0.5).withTimeout(1.5)
    );
  }
}
