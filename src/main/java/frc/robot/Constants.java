package frc.robot;

public class Constants {
    public static class Flywheel {
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    public static class Turret {
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final double gearRatio = 8/1;
    }

    public static class CanIDs {
        // TODO: Intake
        public static final int intakePivotMotorID = 0;
        public static final int intakePivotEncoderID = 0;
        public static final int intakeMotorID = 0;

        // Shooter
        public static final int flywheelMotorID = 41; // Kraken x60 that powers the shooter.
        public static final int shooterHoodMotorID = 0;
        public static final int shooterHoodEncoderID = 0;
        public static final int turretMotorID = 60;
        public static final int turretEncoderID = 0;
    }
}
