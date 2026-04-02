package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterHood extends SubsystemBase {
    private final TalonFX hoodMotor = new TalonFX(Constants.CanIDs.shooterHoodMotorID);

    private PositionVoltage pv = new PositionVoltage(0).withSlot(0);

    private final RobotContainer rob;
    private final CommandSwerveDrivetrain swerve;

    private double degrees = 0;
    private double manualOffset = 0;

    private boolean autoDecap = false;

    private Slot0Configs pidConfigs = new Slot0Configs();

    public ShooterHood(RobotContainer rob) {
        this.rob = rob;
        this.swerve = rob.drivetrain;

        CurrentLimitsConfigs hoodCurrentLimits = new CurrentLimitsConfigs();
        hoodCurrentLimits.StatorCurrentLimit = 15;
        hoodCurrentLimits.StatorCurrentLimitEnable = true;

        // pidConfigs.kP = Constants.Hood.kP;
        // pidConfigs.kI = Constants.Hood.kI;
        // pidConfigs.kD = Constants.Hood.kD;
        // pidConfigs.kS = Constants.Hood.kS;
        // pidConfigs.kV = Constants.Hood.kV;
        // pidConfigs.kA = Constants.Hood.kA;

        SmartDashboard.putNumber("hood p", 0);
        SmartDashboard.putNumber("hood i", 0);
        SmartDashboard.putNumber("hood d", 0);
        SmartDashboard.putNumber("hood s", 0);
        SmartDashboard.putNumber("hood v", 0);
        SmartDashboard.putNumber("hood a", 0);
        SmartDashboard.putNumber("hood setpoint", 0);

        // MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        // mmConfigs.MotionMagicCruiseVelocity = 5;
        // mmConfigs.MotionMagicAcceleration = 10;

        hoodMotor.clearStickyFaults();
        // hoodMotor.getConfigurator().apply(mmConfigs);
        hoodMotor.getConfigurator().apply(pidConfigs);
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        SmartDashboard.putNumber("Set Angles", 0); // need for getNumber
        hoodMotor.setPosition(0);
    }

    public void setRotations(double rotations) {
        // negative sign is only for belted hood
        hoodMotor.setControl(pv.withPosition(-rotations));
    }

    public double getRotations() {
        // negative sign is only for belted hood
        return -hoodMotor.getPosition().getValueAsDouble();
    }

    public void setDegrees() {
        degrees = Math.max(Math.min(Constants.Hood.maxHoodAngle, degrees), 0) + manualOffset;

        double rotations = degrees * Constants.Hood.gearRatio / 360;
        setRotations(rotations);
    }

    public void alignHoodtoHub(double distance) {
        double angle;
        double d = Math.min(distance, 9);

        // NEW CALI (as of March 17)
        if (d <= 5.5) {
            // angle = 0.21 * Math.pow(d, 3) - 2.56 * Math.pow(d, 2) + 13.35 * d - 12.91; // low arc
            angle = 0.06 * Math.pow(d, 3) - 1.25 * Math.pow(d, 2) + 9.82 * d - 10.07;
        } else {
            // copied from old cali
            angle = 0.15 * Math.pow(d, 3) - 2.68 * Math.pow(d, 2) + 16.77 * d - 26.51;
        }
        // if (d <= 3) {
        //     // angle = 0.21 * Math.pow(d, 3) - 2.56 * Math.pow(d, 2) + 13.35 * d - 12.91; // low arc
        //     angle = 0.06 * Math.pow(d, 3) - 1.25 * Math.pow(d, 2) + 9.82 * d - 10.07;
        // } else if (d <= 5){
        //     angle = 0; // FIXME: put smth here so we don't shoot vertically
        // } else {
        //     // copied from old cali
        //     angle = 0.15 * Math.pow(d, 3) - 2.68 * Math.pow(d, 2) + 16.77 * d - 26.51;
        // } 

        angle = Math.max(Math.min(Constants.Hood.maxHubHoodAlign, angle), Constants.Hood.minHoodAngle);
        SmartDashboard.putNumber("HOOD ANGLE", angle);
        degrees = angle;
        setDegrees();
    }

    public void alignHoodtoLob() {
        // double angle = 0;
        // angle = Math.max(Math.min(Constants.Hood.maxHubHoodAlign, angle), 0);
        degrees = 18;
        setDegrees();
    }

    public void changeDegrees(double deg) {
        double rotations = (getDegrees() + deg) * Constants.Hood.gearRatio / 360;
        setRotations(rotations);
    }

    public double getDegrees() {
        return 360 * getRotations() / Constants.Hood.gearRatio;
    }

    public double getCurrent() {
        return hoodMotor.getStatorCurrent().getValueAsDouble();
    }

    public void setManualOffset(double newManualOffset) {
        manualOffset = newManualOffset;
    }

    public double getManualOffset() {
        return manualOffset;
    }

    public void enableAutoDecap() {
        autoDecap = true;
    }

    public void disableAutoDecap() {
        autoDecap = false;
    }

    public void stop() {
        hoodMotor.setControl(new VoltageOut(0));
    }

    public void zeroHood() {
        while (getCurrent() < Constants.Hood.currentWhenStalling) {
            changeDegrees(-2.5);
        }
        this.stop();

        // tell the hood that this is the correct zero position
        hoodMotor.setPosition(0);
        degrees = 0;
    }

    public boolean willGetDecapitated() {
        // If the leftBumper is held down, manual override is triggered, 
        // keeping the hood down until the driver says it's safe to lift.
        boolean manualOverrideDriver = rob.driverController.rightBumper().getAsBoolean(); 
        boolean manualOverrideOperator =  rob.operatorController.leftTrigger().getAsBoolean();

        if (manualOverrideDriver) {
            return true;
        }

        if (manualOverrideOperator){
            return true;
        }

        if (autoDecap) {
            return true;
        }

        Pose2d currentPos = swerve.getPoseMeters();

        double robotRadians = currentPos.getRotation().getRadians();
        Translation2d turretOffset = new Translation2d(Constants.Turret.offsetFromCenter.getX(),
                -Constants.Turret.offsetFromCenter.getY()).rotateBy(new Rotation2d(robotRadians));

        Pose2d turretLocation = new Pose2d(currentPos.getX() + turretOffset.getX(),
                currentPos.getY() + turretOffset.getY(), Rotation2d.fromDegrees(0));

        double x = turretLocation.getX();
        double y = turretLocation.getY();
        double redX = Constants.Hood.trenchRedX;
        double blueX = Constants.Hood.trenchBlueX;
        double offset = Constants.Hood.trenchOffset;

        if (rob.isRed()) {
            return (x < redX + offset && x > redX - offset) && (y < 1.37889 || y > 6.6842892); // should be 0.2 meter off trench for x and 0.1 for y // TODO: CONSTANTS
        } else {
            return (x < blueX + offset && x > blueX - offset) && (y < 1.37889 || y > 6.6842892); // should be 0.5 meter off trench for x and 0.1 for y
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Rotations", getRotations());
        SmartDashboard.putNumber("Hood Degrees", getDegrees());
        SmartDashboard.putNumber("HOOD SIM ANGLE", degrees);
        SmartDashboard.putNumber("Hood", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood Current", getCurrent());
        SmartDashboard.putBoolean("Will get decapitated", willGetDecapitated());

         // for pid tuning
        double p = SmartDashboard.getNumber("hood p", 0);
        double i = SmartDashboard.getNumber("hood i", 0);
        double d = SmartDashboard.getNumber("hood d", 0);
        double s = SmartDashboard.getNumber("hood s", 0);
        double v = SmartDashboard.getNumber("hood v", 0);
        double a = SmartDashboard.getNumber("hood a", 0);

        boolean isChanged = (
            p != pidConfigs.kP ||
            i != pidConfigs.kI ||
            d != pidConfigs.kD ||
            s != pidConfigs.kS ||
            v != pidConfigs.kV ||
            a != pidConfigs.kA);
        
        if (isChanged) {
            pidConfigs.kP = p;
            pidConfigs.kI = i;
            pidConfigs.kD = d;
            pidConfigs.kS = s;
            pidConfigs.kV = v;
            pidConfigs.kA = a;
            hoodMotor.getConfigurator().apply(pidConfigs);
        }

        degrees = SmartDashboard.getNumber("Set Angles", 0);
        setDegrees();
        // double future[] = rob.turret.aLookIntoTheFuture(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        // double sotmV[] = rob.turret.sotmPredictVelocity(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        // double sotmVA[] = rob.turret.sotmPredictVelocityAcceleration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        // double sotmVAI[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        double sotmVI[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
                rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);

        // if (rob.isShooting && !willGetDecapitated()) {
        //     if (rob.turret.isInAllianceZone()) {
        //         alignHoodtoHub(sotmVI[0]);
        //     } else {
        //         alignHoodtoLob();
        //     }
        // } else {
        //     degrees = 0.5;
        //     setDegrees();
        // }
    }
}