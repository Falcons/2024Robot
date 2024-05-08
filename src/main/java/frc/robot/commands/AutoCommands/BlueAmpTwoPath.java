// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueAmpTwoPath extends SequentialCommandGroup {
  /** Creates a new BlueAmpTwoNotePath. */
  public BlueAmpTwoPath(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterPivot shooterpivot, LimelightShooter ls) {
    Trajectory trajectory = PathConstants.blueAmpSideTwoNote;
    
    RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(DriveConstants.kPVel, 0, 0),
                new PIDController(DriveConstants.kPVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain);
    addCommands(
      new InstantCommand(() -> drivetrain.setFieldPath(trajectory), drivetrain),
      new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose()), drivetrain),
      new OneNote(drivetrain, intake, shooter, shooterpivot, ls),
      new ParallelCommandGroup(
        new Extend(intake),
        ramseteCommand).until(intake::hasNote),
      new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
      new Retract(intake)
    );
  }
}
