// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoCommands.OneNoteAuto;
import frc.robot.commands.AutoCommands.OneNoteWithTension;
import frc.robot.commands.AutoCommands.RedAmpSideTwoNote;
import frc.robot.commands.AutoCommands.RedSourceSideTwoNote;
import frc.robot.commands.AutoCommands.BlueAmpSideOneNote;
import frc.robot.commands.AutoCommands.BlueAmpSideTwoNote;
import frc.robot.commands.AutoCommands.BlueSourceRoutines;
import frc.robot.commands.AutoCommands.BlueSourceSideTwoNote;
import frc.robot.commands.AutoCommands.TwoNoteAuto;
import frc.robot.commands.AutoCommands.TwoNoteCentreWithTension;
import frc.robot.commands.ClimbCommands.ClimbManual;
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
      new ClimbManual(climb, 
      () -> operator.getLeftY(), 
      () -> operator.getRightY())
    );

    chooser.setDefaultOption("One Note", new OneNoteWithTension(drivetrain, intake, shooter, shooterpivot, limelightshooter));
    chooser.addOption("Two Note Centre", new TwoNoteCentreWithTension(drivetrain, intake, shooter, shooterpivot, limelightshooter));

    chooser.addOption("Blue Amp Side", new BlueAmpSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    chooser.addOption("Blue Source Side", new BlueSourceSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));

    chooser.addOption("Red Amp Side", new RedAmpSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    chooser.addOption("Red Source Side", new RedSourceSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    chooser.addOption("Blue Source w/Routines", new BlueSourceRoutines(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    Shuffleboard.getTab("Drivetrain").add(chooser);
  }

  public void disabledPeriodic() {
    limelightshooter.setPriorityID();
  }

  private void configureBindings() {
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

    //configs
    driver.a().onTrue(new InstantCommand(drivetrain::invertMotors));
    driver.b().onTrue(new InstantCommand(drivetrain::resetEncoders).ignoringDisable(true));
    driver.x().onTrue(new InstantCommand(drivetrain::FastMode));
    driver.rightTrigger().onTrue(new InstantCommand(drivetrain::SlowMode));
    

    // Intake

    //wheels manual
    operator.x().whileTrue(intake.IntakeNoteCmd(IntakeConstants.intakeSpeed));
    operator.a().whileTrue(intake.EjectNoteCmd(1));
    //setpoint Manual
    operator.povUp().onTrue(new Extend(intake));
    operator.povDown().onTrue(new Retract(intake));
    //extend -> intake -> retract
    operator.rightBumper().onTrue(
      new Extend(intake).until(intake::hasNote)
      .andThen(new Retract(intake)));
      

    // Shooterpivot

    //Shooter adjustment
    driver.povUp().onTrue(new SetShooterTwoPID(shooterpivot, shooterpivot.rawToDegrees(0.95)));
    //driver.povDown().onTrue(new SetShooterFixed(shooterpivot, 36.3));\
    driver.povRight().onTrue(new SetShooterTwoPID(shooterpivot, 42));
    driver.povDown().onTrue(new SetShooterTwoPID(shooterpivot, 36.3));
    operator.povLeft().whileTrue(new SetShooterPosition(shooterpivot, limelightshooter));

    //Shooter instant
    //operator.y().whileTrue(shooter.Shoot(1, 0.95));
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
    return chooser.getSelected();
  }
}
