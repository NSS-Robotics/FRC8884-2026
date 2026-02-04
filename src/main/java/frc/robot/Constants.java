package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static final Pose2d targetPosition = new Pose2d(11.915, 4.017, Rotation2d.fromDegrees(180)); // 180 for red, 0 for blue

    public static class Flywheel {
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class Feeder {
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class Spindexer {
        public static final double kP = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class Turret {
        public static final double kP = 50;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 2.5;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final double gearRatio = 92/10 * 44/14;
        //public static final double gearRatioWithAbsoluteEncoder = 1.5139751553;
        public static final double gearRatioWithAbsoluteEncoder = 0.6605128205;

        //public static final double gearRatioWithAbsoluteEncoder = 92/25 * 14/78;


        public static final Translation2d offsetFromCenter = new Translation2d(0.22, 0.15);
    }

    public static class Hood {
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class Intake {
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class CanIDs {
        // TODO: Intake
        public static final int intakePivotMotorID = 67;
        public static final int intakePivotEncoderID = 67;

        public static final int intakeMotorID = 67;

        // Shooter
        public static final int rightFlywheelMotorID = 60; // Kraken x60 that powers the shooter.
        public static final int leftFlywheelMotorID = 61;

        public static final int shooterHoodMotorID = 62;
        public static final int shooterHoodEncoderID = 67;

        public static final int turretMotorID = 50;
        public static final int turretEncoderID = 51;

        public static final int feederMotorID = 40;

        public static final int spindexerMotorID = 30;
    }
}
