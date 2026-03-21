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

    public ShooterHood(RobotContainer rob) {
        this.rob = rob;
        this.swerve = rob.drivetrain;

        CurrentLimitsConfigs hoodCurrentLimits = new CurrentLimitsConfigs();
        hoodCurrentLimits.StatorCurrentLimit = 15;
        hoodCurrentLimits.StatorCurrentLimitEnable = true;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Hood.kP;
        slot0Configs.kI = Constants.Hood.kI;
        slot0Configs.kD = Constants.Hood.kD;
        slot0Configs.kS = Constants.Hood.kS;
        slot0Configs.kV = Constants.Hood.kV;
        slot0Configs.kA = Constants.Hood.kA;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicCruiseVelocity = 5;
        mmConfigs.MotionMagicAcceleration = 10;

        hoodMotor.clearStickyFaults();
        hoodMotor.getConfigurator().apply(mmConfigs);
        hoodMotor.getConfigurator().apply(slot0Configs);
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        SmartDashboard.putNumber("Set Angles", 0); // need for getNumber
        hoodMotor.setPosition(0);
    }

    public void setRotations(double rotations) {
        // FIXME: negative sign is only for belted hood
        hoodMotor.setControl(pv.withPosition(-rotations));
    }

    public double getRotations() {
        // FIXME: negative sign is only for belted hood
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
        if (d < 5.0) {
            angle = 0.31 * Math.pow(d, 3) - 3.50 * Math.pow(d, 2) + 14.05 * d - 12.41;
        } else {
            // copied from old cali
            angle = 0.15 * Math.pow(d, 3) - 2.68 * Math.pow(d, 2) + 16.77 * d - 26.51;
        }

        // BAD CALI (as of March 16)
        // if (d < 5) {
        //     angle = 0.26 * Math.pow(distance, 3) - 3.04 * Math.pow(distance, 2) + 12.85 * distance - 11.46;
        // } else {
        //     // copied from old cali
        //     angle = 0.15 * Math.pow(d, 3) - 2.68 * Math.pow(d, 2) + 16.77 * d - 26.51;
        // }

        // OLD CALI
        // // double angle = 0.06 * Math.pow(distance, 3) - 0.54 * Math.pow(distance, 2) +
        // // 5.33 * distance - 4.88;
        // double angle;
        // double d = Math.min(distance, 9); 
        // d-=0.1;
        // if (d < 5.1) {
        //     // angle = 1.21 * Math.pow(d, 3) - 9.9 * Math.pow(d, 2) + 28.61 * d - 22.42;
        //     angle = 0.27 * Math.pow(d,3) - 3.08 * Math.pow(d, 2) + 12.92 * d  - 11.22;
        // } else {
        //     angle = 0.15 * Math.pow(d, 3) - 2.68 * Math.pow(d, 2) + 16.77 * d - 26.51;
        // } 
        // // else{
        // //     angle = -0.02 * Math.pow(d, 3) + 0.28 * Math.pow(d, 2) + 0.37 * d + 8.66; // lob
        // // }
    

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

        // degrees = SmartDashboard.getNumber("Set Angles", 0);
        // setDegrees();
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

        // removed because we now use a constant hood angle for lobbing
        // double sotmVILobRight[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.rightCornerRed : Constants.rightCornerBlue);
        // double sotmVILobLeft[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.leftCornerRed : Constants.leftCornerBlue);

        if (!rob.turret.isInAllianceZone()) {
            boolean isFerrying = rob.operatorController.rightTrigger().getAsBoolean();
            if (isFerrying) {
                if (swerve.getPoseMeters().getY() > 4){
                    // alignHoodtoLob(sotmVILobLeft[0]);
                    alignHoodtoLob();
                } else {
                    // alignHoodtoLob(sotmVILobRight[0]);
                    alignHoodtoLob();
                }
            } else {
                degrees = 0.5;
                setDegrees();
            }
        } else {
            alignHoodtoHub(sotmVI[0]);
        }

        if (willGetDecapitated()) {
            degrees = 0.5; 
            setDegrees();
        }
    }
}