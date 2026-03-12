package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    // Red positions
    public static final Pose2d targetPositionRed = new Pose2d(11.915, 4.03, Rotation2d.fromDegrees(180));        
    // public static final Pose2d targetPositionRed = new Pose2d(11.915, 4.03, Rotation2d.fromDegrees(180));        
    public static final Pose2d leftCornerRed = new Pose2d(16, 8 - 0.5, Rotation2d.fromDegrees(180));
    public static final Pose2d rightCornerRed = new Pose2d(16, 0.05 + 0.5, Rotation2d.fromDegrees(180));

    // Blue positions
    public static final Pose2d targetPositionBlue = new Pose2d(4.625, 4.03, Rotation2d.fromDegrees(180)); //this is the correct one
    // public static final Pose2d targetPositionBlue = new Pose2d(4.625, 3.90, Rotation2d.fromDegrees(180));
    public static final Pose2d leftCornerBlue = new Pose2d(0.05, 8 - 0.5, Rotation2d.fromDegrees(180)); 
    public static final Pose2d rightCornerBlue = new Pose2d(0.05, 0.05 + 0.5, Rotation2d.fromDegrees(180)); 

    public static class Dimensions {
        public static final double robotWidth = Units.inchesToMeters(30);
        public static final double robotLength = Units.inchesToMeters(25);
    }

    public static class Flywheel {
        public static final double currentLimit = 40;
        public static final double kP = 0.1; // TODO: tune
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.25;
        public static final double kV = 0.123;
        public static final double kA = 0.0;
        public static final double flywheelRadiusMeters = 0.0508;
    }

    public static class Feeder {
        public static final double currentLimit = 80;
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.7;
        public static final double kV = 0.1;
        public static final double kA = 0.0;
    }

    public static class Spindexer {
        public static final double currentLimit = 60;
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class Turret {
        public static final double currentLimit = 40;
        // public static final double kP = 4; // 6
        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = 0.05;
        public static final double kS = 0.35;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final double gearRatio = 92/10 * 44/14;
        //public static final double gearRatioWithAbsoluteEncoder = 1.5139751553;
        public static final double gearRatioWithAbsoluteEncoder = 0.6605128205;

        // public static final double gearRatioWithAbsoluteEncoder = 92/25 * 14/78;
        public static final double tolerance = 1.2;

        public static final Translation2d offsetFromCenter = new Translation2d(0.22, 0.15);

        public static final double rotationLimit1 = -180.0;
        public static final double rotationLimit2 = 210.0;
    }

    public static class Hood {
        public static final double currentLimit = 15;
        public static final double kP = 15.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double gearRatio = 64.0/14.0 * 172.0/10.0;
        public static final double minHoodAngle = 0.5;
        public static final double maxHoodAngle = 25;
        public static final double currentWhenStalling = 14.7;
        public static final double maxHubHoodAlign = 22;
        // public static final double minHubHoodAlign = 1;
        public static final double irlHoodDegrees = 12.5;

        public static final double trenchRedX = 11.915394; // decapitation
        public static final double trenchBlueX = 4.625594;
        public static final double trenchOffset = 0.35;
    }

    public static class Intake {
        public static final double currentLimit = 80;
        public static final double kP = 0.8;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class IntakePivot {
        public static final double currentLimit = 40;
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
