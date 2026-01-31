package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class Constants {
    public static final Pose2d targetPosition = new Pose2d(11.915, 4.017, null);

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
        public static final double kP = 19;
        public static final double kI = 0.01;
        public static final double kD = 0.2;
        public static final double kS = 2.5;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final double gearRatio = 92/10 * 44/14;
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
        public static final int intakePivotMotorID = 0;
        public static final int intakePivotEncoderID = 0;

        public static final int intakeMotorID = 0;

        // Shooter
        public static final int rightFlywheelMotorID = 60; // Kraken x60 that powers the shooter.
        public static final int leftFlywheelMotorID = 61;

        public static final int shooterHoodMotorID = 62;
        public static final int shooterHoodEncoderID = 0;

        public static final int turretMotorID = 50;
        public static final int turretEncoderID = 0;

        // Feeder/Indexer => drumroll, please... => Finndexer!!!
        public static final int feederMotorID = 40;

        public static final int spindexerMotorID = 30;
    }
}
