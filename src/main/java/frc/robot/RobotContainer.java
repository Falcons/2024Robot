// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoCommands.OneNote;
import frc.robot.commands.AutoCommands.RedAmpBlueSourceOneNoteTaxi;
import frc.robot.commands.AutoCommands.RedAmpSideTwoNote;
import frc.robot.commands.AutoCommands.RedSourceSideTwoNote;
import frc.robot.commands.AutoCommands.RunPath;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.AutoCommands.BlueAmpRedSourceOneNoteTaxi;
import frc.robot.commands.AutoCommands.BlueAmpSidePathPlan;
import frc.robot.commands.AutoCommands.BlueAmpSideTwoNote;
import frc.robot.commands.AutoCommands.BlueAmpTwoPath;
import frc.robot.commands.AutoCommands.BlueSourceRoutines;
import frc.robot.commands.AutoCommands.BlueSourceSideTwoNote;
import frc.robot.commands.AutoCommands.TwoNoteCentreWithTension;
import frc.robot.commands.ClimbCommands.ClimbManual;
import frc.robot.commands.DriveCommands.ArcadeDriveCmd;
import frc.robot.commands.DriveCommands.CentretoNote;
import frc.robot.commands.DriveCommands.CentretoSpeakerTele;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.ShooterCommands.SetShooterFF;
import frc.robot.commands.ShooterCommands.SetShooterLL;
import frc.robot.commands.ShooterCommands.SetShooterShuffleboard;
import frc.robot.commands.ShooterCommands.SetShooterTwoPID;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
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
  private final LEDs leds = new LEDs();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Trajectory> pathChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    Command resetToBlueCentre = new InstantCommand(() -> drivetrain.resetOdometry(DriveConstants.blueSubWooferCentre)).ignoringDisable(true);
    Command resetToBlueAmp = new InstantCommand(() -> drivetrain.resetOdometry(DriveConstants.blueSubWooferAmp)).ignoringDisable(true);
    
    SmartDashboard.putData("Reset Blue Centre", resetToBlueCentre);
    SmartDashboard.putData("Reset Blue Amp", resetToBlueAmp);

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
      //LED update
    leds.setDefaultCommand(new SetLEDs(leds, intake, shooter).ignoringDisable(true));

    pathChooser.setDefaultOption("Blue Amp Two", PathConstants.blueAmpSideTwoNote);
    pathChooser.addOption("All Mid", PathConstants.blueAllMidNotes);
    pathChooser.addOption("Red Amp Two", PathConstants.redAmpTwoSide);

    autoChooser.setDefaultOption("One Note", new OneNote(drivetrain, intake, shooter, shooterpivot, limelightshooter));
    autoChooser.addOption("Blue Amp/Red Source One Note", new BlueAmpRedSourceOneNoteTaxi(drivetrain, intake, shooter, shooterpivot, limelightshooter));
    autoChooser.addOption("Red Amp/Blue Source One Note", new RedAmpBlueSourceOneNoteTaxi(drivetrain, intake, shooter, shooterpivot, limelightshooter));

    autoChooser.addOption("Two Note Centre", new TwoNoteCentreWithTension(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));

    autoChooser.addOption("Blue Amp Side", new BlueAmpSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    autoChooser.addOption("Blue Source Side", new BlueSourceSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));

    autoChooser.addOption("Red Amp Side", new RedAmpSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    autoChooser.addOption("Red Source Side", new RedSourceSideTwoNote(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    autoChooser.addOption("Blue Source w/Routines", new BlueSourceRoutines(drivetrain, intake, shooter, shooterpivot, limelightshooter, limelightintake));
    autoChooser.addOption("Blue Amp Path Plan", new BlueAmpTwoPath(drivetrain, intake, shooter, shooterpivot, limelightshooter));

    Shuffleboard.getTab("Auto").add(autoChooser);
    Shuffleboard.getTab("Auto").add(pathChooser);

    SmartDashboard.putData("Drive", drivetrain);
    SmartDashboard.putData("Intake", intake);
    SmartDashboard.putData("Pivot", shooterpivot);
    SmartDashboard.putData("Shooter", shooter);
    SmartDashboard.putData("Climb", climb);
  }

  public void disabledPeriodic() {
    limelightshooter.setPriorityID();
  }

  private void configureBindings() {
    // Drivetrain

    //centring
    driver.leftBumper().onTrue(new CentretoNote(drivetrain, limelightintake));
    driver.rightBumper().whileTrue(new CentretoSpeakerTele(drivetrain, limelightshooter, () -> -driver.getLeftY()));
    //configs
    driver.a().onTrue(new InstantCommand(drivetrain::invertMotors));
    driver.x().onTrue(new InstantCommand(drivetrain::FastMode));
    driver.rightTrigger().onTrue(new InstantCommand(drivetrain::SlowMode));


    // Intake

    //wheels manual
    operator.x().whileTrue(intake.IntakeNoteCmd(IntakeConstants.intakeSpeed));
    operator.a().whileTrue(intake.EjectNoteCmd(IntakeConstants.outtakeSpeed));
    //setpoint Manual
    operator.povUp().onTrue(new Extend(intake));
    operator.povDown().onTrue(new Retract(intake));
    //extend -> intake -> retract
    operator.rightBumper().onTrue(
      new Extend(intake).until(intake::hasNote)
      .andThen(new Retract(intake)));
      

    // Shooterpivot

    //Shooter adjustment
    driver.povUp().onTrue(new SetShooterTwoPID(shooterpivot, 56.93));
    driver.povRight().onTrue(new SetShooterTwoPID(shooterpivot, 42));
    driver.povDown().onTrue(new SetShooterTwoPID(shooterpivot, 36.3));

    operator.povLeft().whileTrue(new SetShooterLL(shooterpivot, limelightshooter));
    operator.povRight().whileTrue(new SetShooterShuffleboard(shooterpivot));
    operator.y().whileTrue(new SetShooterFF(shooterpivot));
    operator.back().onTrue(new InstantCommand(() -> shooterpivot.setVoltage(0), shooterpivot));

    //Amp Mode
  /*
    operator.leftBumper().whileTrue(
      new SetShooterTwoPID(shooterpivot, 64)
      .alongWith(shooter.Shoot(0.10, 0.10))); //0.1325
  */
    //shooter variable
    operator.leftTrigger(0.3).whileTrue(shooter.Shoot(0.5, 0.5).unless(operator.leftTrigger(0.9)));
    operator.leftTrigger(0.9).whileTrue(shooter.Shoot(1, 0.95));

/*
    //Subsystem Characterization
    driver
      .a()
      .and(driver.leftBumper())
      .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driver
      .b()
      .and(driver.leftBumper())
      .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driver
      .x()
      .and(driver.leftBumper())
      .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driver
      .y()
      .and(driver.leftBumper())
      .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  */
  }

  public Command getAutonomousCommand() {
/*
    Command runTraj = Commands.runOnce(() -> drivetrain.resetOdometry(drivetrain.closeNoteBlue.getInitialPose()))
      .andThen(drivetrain.ramseteCommand)
      .andThen(Commands.runOnce(() -> drivetrain.tankDriveVolts(0, 0)));
*/
    //return runTraj;
    return autoChooser.getSelected();
    //return new RunPath(drivetrain, pathChooser.getSelected());
    //return new PathPlannerAuto("Test Auto");
  }
}
