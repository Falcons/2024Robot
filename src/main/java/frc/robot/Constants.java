// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DriveConstants {
        //CAN Ids
        public static final int frontRightID = 2;
        public static final int backRightID = 4;
        public static final int frontLeftID = 1;
        public static final int backLeftID = 3;
        public static final int pigeonID = 12;

        //Encoder Conversion Ratios
        public static final double RevToMetre = 1188.0 * Math.PI / 82991.96;
        public static final double RPMToMetresPerSecond = 1188.0 * Math.PI / 4979517.6;

        //SysId Values
        public static final double ksVolts = 0.21888; //Garage: 0.21888, Carpet: 0.21599
        public static final double kvVoltSecondsPerMeter = 2.6459; //Garage: 2.6459, Carpet: 2.4096
        public static final double kaVoltSecondsSquaredPerMeter = 0.62854; //Garage: 0.62854, Carpet: 0.87369
        public static final double kPVel = 3.739; //Garage: 3.739, Carpet: 3.538
        
        //Ramsetecommand configs
        public static final double kTrackwidthMeters = 0.683;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        //Robot Poses
        public static final double kRobotLength = Units.inchesToMeters(37.5);
        public static final Pose2d blueSubWooferCentre = new Pose2d(Units.inchesToMeters(36.37) + kRobotLength / 2.0, Units.inchesToMeters(218.42), new Rotation2d());
        public static final Pose2d blueSubWooferAmpSide = new Pose2d(Units.inchesToMeters(36.37), Units.inchesToMeters(218.42), new Rotation2d());

        //Note Poses
        public static final Pose2d noteBlueCloseAmp = new Pose2d(Units.inchesToMeters(114.0), Units.inchesToMeters(275.42), new Rotation2d());
        public static final Pose2d noteBlueCentreNote = new Pose2d(Units.inchesToMeters(114.0), Units.inchesToMeters(218.42), new Rotation2d());
        public static final Pose2d noteBlueCloseSource = new Pose2d(Units.inchesToMeters(114.0), Units.inchesToMeters(161.42), new Rotation2d());

        public static final Pose2d noteFarAmp1 = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(293.64), new Rotation2d());
        public static final Pose2d noteFarAmp2 = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(227.64), new Rotation2d());
        public static final Pose2d noteFarCentre = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(161.64), new Rotation2d());
        public static final Pose2d noteFarSource2 = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(95.64), new Rotation2d());
        public static final Pose2d noteFarSource1 = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(29.64), new Rotation2d());
    }

    public static final class IntakeConstants {
        //CAN/DIO Ids
        public static final int pivotID = 10;
        public static final int wheelID = 11;
        public static final int intakeBottomLimitDIOPort = 3;

        //Angle in rotations (raw encoder values)
        public static final double intakeOutAngle = 0.08; //0.08
        public static final double intakeInAngle = 0.657;

        //Speed constants
        public static final double intakeSpeed = 0.5;
        public static final double outtakeSpeed = 1.0;
    }

    public static final class ShooterConstants {
        //CAN/DIO Ids
        public static final int leftFlywheelID = 6;
        public static final int rightFlywheelID = 5;
        public static final int pivotID = 7;
        public static final int pivotBottomLimitDIOPort = 0;

        //Soft Limits
        public static final Rotation2d pivotUpperLimit = Rotation2d.fromDegrees(59.1);
        public static final Rotation2d pivotLowerLimit = Rotation2d.fromDegrees(36.33);

        //Encoder Conversion Ratios & Configs
        public static final double degreesToRadians = Math.PI / 180.0;
        public static final double radiansToDegrees = 180.0 / Math.PI;
        public static final double rotationsToDegrees = 360;
        public static final double rotationsToRadians = 2 * Math.PI;
        public static final Rotation2d pivotZeroOffset = Rotation2d.fromDegrees(285.07);

        //Integrated PID and FF (unused)
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;

        public static final double maxSpeed = 0;
        public static final double maxAccel = 0;    
    }

    public static final class ClimbConstants {
        //CAN/DIO Ids
        public static final int leftClimbID = 9;
        public static final int rightClimbID = 8;
        public static final int leftClimbLimitDIOPort = 1;
        public static final int rightClimbLimitDIOPort = 2;
    }
}
