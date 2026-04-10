package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    // Red positions
    public static final Pose2d targetPositionRed = new Pose2d(11.915, 4.03, Rotation2d.fromDegrees(180));
    //public static final Pose2d targetPositionRed = new Pose2d(11.915, 4.20, Rotation2d.fromDegrees(180));
    //public static final Pose2d targetPositionRed = new Pose2d(11.515, 4.20, Rotation2d.fromDegrees(180));

    public static final Pose2d leftCornerRed = new Pose2d(16 - 3, 8 - 2, Rotation2d.fromDegrees(180));
    public static final Pose2d rightCornerRed = new Pose2d(16 - 3, 0.05 + 2, Rotation2d.fromDegrees(180));

    // Blue positions
    public static final Pose2d targetPositionBlue = new Pose2d(4.625, 4.03, Rotation2d.fromDegrees(180)); //this is the correct one
    //public static final Pose2d targetPositionBlue = new Pose2d(4.625, 3.95, Rotation2d.fromDegrees(180));
    public static final Pose2d leftCornerBlue = new Pose2d(0.05 + 3, 8 - 2, Rotation2d.fromDegrees(180)); 
    public static final Pose2d rightCornerBlue = new Pose2d(0.05 + 3, 0.05 + 2, Rotation2d.fromDegrees(180)); 

    // public static final double fieldXMeter = 16.540988; // FIXME: ADJUST WHEN TESTING AUTOE FOR BLUE
    public static final double fieldXMeter = 16.1; // we shall see...
    public static final double fieldYMeter = 8.142656;

    public static class Dimensions {
        public static final double robotWidth = Units.inchesToMeters(30);
        public static final double robotLength = Units.inchesToMeters(25);
    }

    public static class Flywheel {
        public static final double currentLimit = 80;
        public static final double supplyLimit = 50;

        public static final double kP = 12;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 4.0;
        public static final double kV = 0.3;
        public static final double kA = 0.0;
        public static final double flywheelRadiusMeters = 0.0508;
    }

    public static class Feeder {
        public static final double currentLimit = 60; 
        public static final double supplyLimit = 60;
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.7;
        public static final double kV = 0.1;
        public static final double kA = 0.0;
    }

    public static class Spindexer {
        public static final double currentLimit = 70;
        public static final double supplyLimit = 50;

        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class Turret {
        public static final double currentLimit = 40;
        public static final double kP_0 = 4000.0;
        public static final double kI_0 = 0.0;
        public static final double kD_0 = 150.0;
        public static final double kS_0 = 0.0;
        public static final double kV_0 = 0.0;
        public static final double kA_0 = 0.0;

        public static final double kP_1 = 250.0;
        public static final double kI_1 = 0.0;
        public static final double kD_1 = 20.0;
        public static final double kS_1 = 0.0;
        public static final double kV_1 = 0.0;
        public static final double kA_1 = 0.0;

        public static final double gearRatio = 92/10 * 44/14;
        public static final double gearRatioWithAbsoluteEncoder = 0.6605128205;
        public static final double motorToEncoderRatio = 43.77551020493131;

        public static final double tolerance = 1.2;

        public static final Translation2d offsetFromCenter = new Translation2d(0.22, 0.15);

        public static final double rotationLimit1 = -180.0;
        public static final double rotationLimit2 = 200.0;

        // to zero turret encoder:
        // - point turret forward
        // - set magnet offset to 0 in tuner
        // - get value from elastic
        // - multiply by gearRatioWithAbsoluteEncoder
        // - set it here
        public static final double encoderOffset = 0.5948907162657471;
    }

    public static class Hood {
        public static final double currentLimit = 30;
        public static final double upKP = 20.0;
        public static final double upKI = 0.0;
        public static final double upKD = 0.0;
        public static final double upKS = 0.0;
        public static final double upKV = 0.0;
        public static final double upKA = 0.0;

        public static final double downKP = 8.884;
        public static final double downKI = 0.0;
        public static final double downKD = 0.0;
        public static final double downKS = 0.0;
        public static final double downKV = 0.0;
        public static final double downKA = 0.0;

        public static final double gearRatio = 16.0/14.0 * 172.0/10.0;
        public static final double minHoodAngle = 0.5;
        public static final double maxHoodAngle = 25;
        public static final double currentWhenStalling = 14.7;
        public static final double maxHubHoodAlign = 22;
        public static final double irlHoodDegrees = 12.5;

        public static final double trenchRedX = 11.915394; // decapitation
        public static final double trenchBlueX = 4.625594;
        public static final double trenchOffset = 0.35;
    }

    public static class Intake {
        public static final double currentLimit = 80;
        public static final double supplyLimit = 60;
        public static final double kP = 0.8;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class IntakePivot {
        public static final double currentLimit = 30;
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class CanIDs {
        // Intake
        public static final int intakePivotMotorID = 20;
        public static final int intakeMotorID = 21;

        // Shooter
        public static final int rightFlywheelMotorID = 60; // Kraken x60 that powers the shooter.
        public static final int leftFlywheelMotorID = 61;

        public static final int shooterHoodMotorID = 62;

        public static final int turretMotorID = 50;
        public static final int turretEncoderID = 51;

        public static final int feederMotorID = 40;

        public static final int spindexerMotorID = 30;
    }
}
