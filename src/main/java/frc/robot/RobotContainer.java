// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoCommands.OneNoteWithTension;
import frc.robot.commands.AutoCommands.RedAmpSideTwoNote;
import frc.robot.commands.AutoCommands.RedSourceSideTwoNote;
import frc.robot.commands.AutoCommands.BlueAmpSideOneNote;
import frc.robot.commands.AutoCommands.BlueAmpSideTwoNote;
import frc.robot.commands.AutoCommands.BlueSourceSideTwoNote;
import frc.robot.commands.AutoCommands.TwoNoteCentreWithTension;
import frc.robot.commands.ClimbCommands.ClimbUp;
import frc.robot.commands.DriveCommands.ArcadeDriveCmd;
import frc.robot.commands.DriveCommands.CentretoNote;
import frc.robot.commands.DriveCommands.CentretoSpeaker;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.ShooterCommands.Down;
import frc.robot.commands.ShooterCommands.SetShooterFixed;
import frc.robot.commands.ShooterCommands.SetShooterPosition;
import frc.robot.commands.ShooterCommands.SetShooterTwoPID;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.commands.ShooterCommands.Up;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Climb climb = new Climb();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final ShooterPivot shooterpivot = new ShooterPivot();
  private final LimelightIntake limelightintake = new LimelightIntake();
  private final LimelightShooter limelightshooter = new LimelightShooter();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

      //Arcade Drive

    drivetrain.setDefaultCommand(
      new ArcadeDriveCmd(drivetrain, 
      () -> -driver.getLeftY(), 
      () -> -driver.getRightX()));

      //Climb Joysticks

    climb.setDefaultCommand(
      new ClimbUp(climb, 
      () -> operator.getLeftY(), 
      () -> operator.getRightY())
    );

    ;
    //chooser.addOption("Centre Two Note", new TwoNoteAuto(drivetrain, intake, shooter, shooterpivot, limelightshooter));
    chooser.addOption("Blue Amp Side", new BlueAmpSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    chooser.addOption("Blue Source Side", new BlueSourceSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    chooser.addOption("Red Amp Side", new RedAmpSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    chooser.addOption("Red Source Side", new RedSourceSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    chooser.addOption("One Note With Tension", new OneNoteWithTension(drivetrain, intake, shooter, shooterpivot, limelightshooter));
    chooser.addOption("Two Note Centre W/Tension", new TwoNoteCentreWithTension(drivetrain, intake, shooter, shooterpivot, limelightshooter));
    Shuffleboard.getTab("Drivetrain").add(chooser);
    
  }

  public void disabledPeriodic() {
    limelightshooter.setPriorityID();
  }

  private void configureBindings() {
    shooterpivot.stopShooterPivot();
/*
    //Wheel Testing
    driver.a().whileTrue(drivetrain.runFrontLeft());
    driver.b().whileTrue(drivetrain.runBackLeft());
    driver.x().whileTrue(drivetrain.runFrontRight());
    driver.y().whileTrue(drivetrain.runBackRight());
*/
    // Drivetrain

    //centring
    driver.leftBumper().onTrue(new CentretoNote(drivetrain, limelightintake));
    driver.rightBumper().whileTrue(new CentretoSpeaker(drivetrain, limelightshooter));

    //invert
    driver.a().onTrue(new InstantCommand(drivetrain::invertMotors));

    //reset encoder
    driver.b().onTrue(new InstantCommand(drivetrain::resetEncoders).ignoringDisable(true));

    //fast and slow
    driver.x().onTrue(new InstantCommand(drivetrain::FastMode));
    driver.rightTrigger().onTrue(new InstantCommand(drivetrain::SlowMode));
    
    // Intake

    //intake wheels manual
    operator.x().whileTrue(intake.IntakeNoteCmd(IntakeConstants.intakeSpeed));
    operator.a().whileTrue(intake.EjectNoteCmd(1));
    //intake Setpoint Manual
    operator.povUp().onTrue(new Extend(intake));
    operator.povDown().onTrue(new Retract(intake));
    //extend -> intake -> retract
    operator.rightBumper().onTrue(
      new Extend(intake).until(intake::hasNote)
      .andThen(new Retract(intake)));
      
    // Shooterpivot

    //Shooter adjustment
    driver.povUp().onTrue(new SetShooterTwoPID(shooterpivot, shooterpivot.rawToDegrees(0.95)));
    driver.povDown().onTrue(new SetShooterFixed(shooterpivot, 36.3));
    operator.povLeft().whileTrue(new SetShooterPosition(shooterpivot, limelightshooter));

    //Shooter instant
    operator.y().whileTrue(shooter.Shoot(1, 0.95));
    //shooter variable
    operator.leftTrigger(0.3).whileTrue(shooter.Shoot(0.5, 0.5).unless(operator.leftTrigger(0.9)));
    operator.leftTrigger(0.9).whileTrue(shooter.Shoot(1, 0.95));

    //Shooter pivot manual
    //operator.povRight().whileTrue(new Down(shooterpivot, 0.1));
    //operator.povLeft().whileTrue(new Up(shooterpivot, 0.1));
/*
    //Shooter Characterization
    operator.a().and(operator.rightBumper())
      .onTrue(shooterpivot.sysIdQuasistatic(Direction.kForward)
      .until(shooterpivot::getSoftLowerLimit));

    operator.b().and(operator.rightBumper())
      .onTrue(shooterpivot.sysIdQuasistatic(Direction.kReverse)
      .until(shooterpivot::getSoftUpperLimit));

    operator.x().and(operator.rightBumper())
      .onTrue(shooterpivot.sysIdDynamic(Direction.kForward)
      .until(shooterpivot::getSoftLowerLimit));

    operator.y().and(operator.rightBumper())
      .onTrue(shooterpivot.sysIdDynamic(Direction.kReverse)
      .until(shooterpivot::getSoftUpperLimit));
*/
  }

  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ks,
                DriveConstants.kv,
                DriveConstants.ka),
            DriveConstants.kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(
                DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory speakerCenter =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), drivetrain.getPose().getRotation()),
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(6, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
          speakerCenter,
          drivetrain::getPose,
          new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
          DriveConstants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(DriveConstants.kPVel, 0, 0),
          new PIDController(DriveConstants.kPVel, 0, 0),
          drivetrain::tankDriveVolts,
          drivetrain);
    
    // return chooser.getSelected();
    return Commands.runOnce(() -> drivetrain.updateOdometry()).andThen(ramseteCommand).andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}
