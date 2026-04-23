package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
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

    // private Slot0Configs tuningPIDConfigs = new Slot0Configs();
    private Slot1Configs upPIDConfigs = new Slot1Configs();
    private Slot2Configs downPIDConfigs = new Slot2Configs();

    public ShooterHood(RobotContainer rob) {
        this.rob = rob;
        this.swerve = rob.drivetrain;

        CurrentLimitsConfigs hoodCurrentLimits = new CurrentLimitsConfigs();
        hoodCurrentLimits.StatorCurrentLimit = 21;
        hoodCurrentLimits.StatorCurrentLimitEnable = true;

        upPIDConfigs.kP = Constants.Hood.upKP;
        upPIDConfigs.kI = Constants.Hood.upKI;
        upPIDConfigs.kD = Constants.Hood.upKD;
        upPIDConfigs.kS = Constants.Hood.upKS;
        upPIDConfigs.kV = Constants.Hood.upKV;
        upPIDConfigs.kA = Constants.Hood.upKA;

        downPIDConfigs.kP = Constants.Hood.downKP;
        downPIDConfigs.kI = Constants.Hood.downKI;
        downPIDConfigs.kD = Constants.Hood.downKD;
        downPIDConfigs.kS = Constants.Hood.downKS;
        downPIDConfigs.kV = Constants.Hood.downKV;
        downPIDConfigs.kA = Constants.Hood.downKA;

        SmartDashboard.putNumber("hood p", 0);
        SmartDashboard.putNumber("hood i", 0);
        SmartDashboard.putNumber("hood d", 0);
        SmartDashboard.putNumber("hood s", 0);
        SmartDashboard.putNumber("hood v", 0);
        SmartDashboard.putNumber("hood a", 0);
        SmartDashboard.putNumber("hood setpoint", 0);

        hoodMotor.clearStickyFaults();
        // hoodMotor.getConfigurator().apply(tuningPIDConfigs);
        hoodMotor.getConfigurator().apply(upPIDConfigs);
        hoodMotor.getConfigurator().apply(downPIDConfigs);
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        SmartDashboard.putNumber("Set Angles", 0); // need for getNumber
        hoodMotor.setPosition(0);
    }

    public void setRotations(double rotations) {
        int slot = 1;
        if (rotations < getRotations()) {
            slot = 2;
        }
        hoodMotor.setControl(pv.withPosition(-rotations).withSlot(slot));
    }

    public double getRotations() {
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

        // if (d <= 5.5) {
        //     // angle = 0.138*Math.pow(d, 4) - 1.89*Math.pow(d, 3) + 8.48*Math.pow(d, 2) - 11.1*d + 4.53;
        //     // angle = 0.0656*Math.pow(d, 4) - 0.862*Math.pow(d, 3) + 3.39*Math.pow(d, 2) - 1.1*d -1.45;
        //     angle = 0.102 * Math.pow(d, 3) - 1.32 *  Math.pow(d,2) + 7.54 * d - 6.5;

        // } else {
        //     angle = 0.15 * Math.pow(d, 3) - 2.68 * Math.pow(d, 2) + 16.77 * d - 26.51;
        // }

        angle = -0.0104 * Math.pow(d, 4) + 0.231 * Math.pow(d, 3)  - 1.87 * Math.pow(d, 2) + 8.46 * d - 7.02;
        //offset for new hood being higher
        angle = angle - 1.0;

        angle = Math.max(Math.min(Constants.Hood.maxHubHoodAlign, angle), Constants.Hood.minHoodAngle);
        SmartDashboard.putNumber("HOOD ANGLE", angle);
        degrees = angle;
        setDegrees();
    }

    public void alignHoodtoLob() {
        degrees = 21;
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

        if (manualOverrideDriver || manualOverrideOperator || autoDecap) {
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
        // double p = SmartDashboard.getNumber("hood p", 0);
        // double i = SmartDashboard.getNumber("hood i", 0);
        // double d = SmartDashboard.getNumber("hood d", 0);
        // double s = SmartDashboard.getNumber("hood s", 0);
        // double v = SmartDashboard.getNumber("hood v", 0);
        // double a = SmartDashboard.getNumber("hood a", 0);

        // boolean isChanged = (
        //     p != tuningPIDConfigs.kP ||
        //     i != tuningPIDConfigs.kI ||
        //     d != tuningPIDConfigs.kD ||
        //     s != tuningPIDConfigs.kS ||
        //     v != tuningPIDConfigs.kV ||
        //     a != tuningPIDConfigs.kA);
        
        // if (isChanged) {
        //     tuningPIDConfigs.kP = p;
        //     tuningPIDConfigs.kI = i;
        //     tuningPIDConfigs.kD = d;
        //     tuningPIDConfigs.kS = s;
        //     tuningPIDConfigs.kV = v;
        //     tuningPIDConfigs.kA = a;
        //     hoodMotor.getConfigurator().apply(tuningPIDConfigs);
        // }

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
                rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue, true);

        if (rob.isShooting && !willGetDecapitated()) {
            if (rob.turret.isInAllianceZone()) {
                alignHoodtoHub(sotmVI[0]);
            } else {
                alignHoodtoLob();
            }
        } else {
            degrees = 0.5;
            setDegrees();
        }
    }
}