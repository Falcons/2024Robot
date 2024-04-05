// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.DriveStraight;
import frc.robot.commands.IntakeCommands.EjectNote;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.ShooterCommands.SetShooterLL;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

/** Collection of Auto Command Groups*/
public class AutoRoutines {
    public static class ShootWithLimelight extends ParallelRaceGroup {
        public ShootWithLimelight(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterPivot shooterpivot, LimelightShooter ls) {
            addCommands(
                new SetShooterLL(shooterpivot, ls),
                new ParallelCommandGroup(
                    new Shoot(shooter, 1, 0.95).withTimeout(3),
                    new SequentialCommandGroup(
                    new WaitCommand(1.5), 
                    new EjectNote(intake, 1).withTimeout(1.5)
                    )
                )
            );
        }
    }

    public static class DriveStraightAndPickup extends SequentialCommandGroup {
  
        public DriveStraightAndPickup(Drivetrain drivetrain, Intake intake) {
            addCommands(
                new ParallelCommandGroup(
                    new Extend(intake),
                    new DriveStraight(drivetrain, 0.5)
                ).until(intake::hasNote),
                new Retract(intake)
            );
        }
    }
}
