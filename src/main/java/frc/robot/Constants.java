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
        public static final int frontRightID = 2;
        public static final int backRightID = 4;

        public static final int frontLeftID = 1;
        public static final int backLeftID = 3;
        
        public static final double RevToMetre = 1188.0 * Math.PI / 82991.96;
        public static final double RPMToMetresPerSecond = 1188.0 * Math.PI / 4979517.6;

        public static final int pigeonID = 12;

        public static final double ksVolts = 0.21888;
        public static final double kvVoltSecondsPerMeter = 2.6459;
        public static final double kaVoltSecondsSquaredPerMeter = 0.62854;

        public static final double kPVel = 3.739; //0.27668
        
        public static final double kTrackwidthMeters = 0.683;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kRobotLength = Units.inchesToMeters(37.5);

        public static final Pose2d blueSubWooferCentre = new Pose2d(Units.inchesToMeters(36.37) + kRobotLength / 2.0, Units.inchesToMeters(218.42), new Rotation2d());
        public static final Pose2d blueSubWooferAmpSide = new Pose2d(Units.inchesToMeters(36.37), Units.inchesToMeters(218.42), new Rotation2d());

        public static final Pose2d blueCloseAmp = new Pose2d(Units.inchesToMeters(114.0), Units.inchesToMeters(275.42), new Rotation2d());
        public static final Pose2d blueCentreNote = new Pose2d(Units.inchesToMeters(114.0), Units.inchesToMeters(218.42), new Rotation2d());
        public static final Pose2d blueCloseSource = new Pose2d(Units.inchesToMeters(114.0), Units.inchesToMeters(161.42), new Rotation2d());

        public static final Pose2d blueFarAmp1 = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(293.64), new Rotation2d());
        public static final Pose2d blueFarAmp2 = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(227.64), new Rotation2d());
        public static final Pose2d blueFarCentre = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(161.64), new Rotation2d());
        public static final Pose2d blueFarSource2 = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(95.64), new Rotation2d());
        public static final Pose2d blueFarSource1 = new Pose2d(Units.inchesToMeters(324.6), Units.inchesToMeters(29.64), new Rotation2d());
    }

    public static final class IntakeConstants {
        public static final int pivotID = 10;
        public static final int wheelID = 11;

        public static final double intakeOutAngle = 0.08; //0.08
        public static final double intakeInAngle = 0.657;

        public static final int intakeBottomLimit = 3;

        public static final double intakeSpeed = 0.5;
    }

    public static final class ShooterConstants {
        public static final int leftFlywheelID = 6;
        public static final int rightFlywheelID = 5;
        public static final int pivotID = 7;

        public static final int pivotBottomLimitPort = 0;

        public static final double speakerDepth = 0.89;

        //public static final double pivotUpperLimit = 54.77; //raw: 0.944
        public static final Rotation2d pivotUpperLimit = Rotation2d.fromDegrees(59.1);
        //public static final double pivotLowerLimit = 36.41; //raw: 0.893
        public static final Rotation2d pivotLowerLimit = Rotation2d.fromDegrees(36.33);

        public static final double degreesToRadians = Math.PI / 180.0;
        public static final double radiansToDegrees = 180.0 / Math.PI;
        public static final double rotationsToDegrees = 360;
        public static final double rotationsToRadians = 2 * Math.PI;
        public static final Rotation2d pivotZeroOffset = Rotation2d.fromDegrees(285.07);

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;

        public static final double maxSpeed = 0;
        public static final double maxAccel = 0;    

        public static final int priorityid = 7;
    }

    public static final class ClimbConstants {
        public static final int leftClimbID = 9;
        public static final int rightClimbID = 8;

        public static final int leftClimbLimitPort = 1;
        public static final int rightClimbLimitPort = 2;
    }
}
