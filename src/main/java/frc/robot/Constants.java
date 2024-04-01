// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class DriveConstants {
        public static final int frontRightID = 2;
        public static final int backRightID = 4;

        public static final int frontLeftID = 1;
        public static final int backLeftID = 3;
        
        public static final double RevToMetre = 1188.0 * Math.PI / 82991.96;
        public static final double RPMToMetresPerSecond = 1188.0 * Math.PI / 4979517.6;

        public static final int pigeonID = 12;

        public static final double ksVolts = 0;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;

        public static final double kPVel = 0;
        
        public static final double kTrackwidthMeters = 0.683;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

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
