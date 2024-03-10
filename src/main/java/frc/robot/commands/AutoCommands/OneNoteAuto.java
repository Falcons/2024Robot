// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.DriveStraight;
import frc.robot.commands.IntakeCommands.EjectNote;
import frc.robot.commands.ShooterCommands.SetShooterFixed;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteAuto extends SequentialCommandGroup {
  public OneNoteAuto(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterPivot shooterpivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterFixed(shooterpivot, 0.967).until(() -> shooterpivot.getThruBore() > 0.96),
      new ParallelCommandGroup(
        new Shoot(shooter, 1, 0.95).withTimeout(3),
        new SequentialCommandGroup(
          new WaitCommand(2), 
          new EjectNote(intake, 1).withTimeout(2))),
      new DriveStraight(drivetrain, 0.5).withTimeout(3)
    );
  }
}
