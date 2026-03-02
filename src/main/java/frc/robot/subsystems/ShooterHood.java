package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

        hoodMotor.clearStickyFaults();
        hoodMotor.getConfigurator().apply(slot0Configs);
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        SmartDashboard.putNumber("Set Angles", 0); // need for getNumber
        hoodMotor.setPosition(0);
    }

    public void setRotations(double rotations) {
        hoodMotor.setControl(pv.withPosition(rotations));
    }

    public double getRotations() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public void setDegrees() {
        degrees = Math.max(Math.min(Constants.Hood.maxHoodAngle, degrees), 0)+manualOffset;

        double rotations = degrees * Constants.Hood.gearRatio / 360;
        setRotations(rotations);
    }

    public void alignHoodtoHub(double distance){
        // double angle = 0.06 * Math.pow(distance, 3) - 0.54 * Math.pow(distance, 2) + 5.33 * distance - 4.88;
        double angle;
        if (distance < 4.49) {
            angle = 1.21 * Math.pow(distance, 3) - 9.9 * Math.pow(distance, 2) + 28.61 * distance - 22.42;
        } else{
            angle = -1.1 * Math.pow(distance, 2) + 13.5 * distance - 24.47;
        }
        angle = Math.max(Math.min(Constants.Hood.maxHubHoodAlign, angle), Constants.Hood.minHoodAngle);
        SmartDashboard.putNumber("HOOD ANGLE", angle);
        degrees = angle;
        setDegrees();
    }

    public void alignHoodtoLob(double distance){
        double angle = 0;
        angle = Math.max(Math.min(Constants.Hood.maxHubHoodAlign, angle), 0);
        degrees = angle;
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

    public void stop() {
        hoodMotor.setControl(new VoltageOut(0));
    }   

    public void zeroHood() {
        while (getCurrent() < Constants.Hood.currentWhenStalling) {
            changeDegrees(-1);
        }
        this.stop();

        // tell the hood that this is the correct zero position
        hoodMotor.setPosition(0);
        degrees = 0;
    }

    public boolean willGetDecapitated(){
        Pose2d currentPos = swerve.getPoseMeters();
        
        double robotRadians = currentPos.getRotation().getRadians(); 
        Translation2d turretOffset = new Translation2d(Constants.Turret.offsetFromCenter.getX(), -Constants.Turret.offsetFromCenter.getY()).rotateBy(new Rotation2d(robotRadians));

        Pose2d turretLocation = new Pose2d(currentPos.getX() + turretOffset.getX(), currentPos.getY() + turretOffset.getY(), Rotation2d.fromDegrees(0));

        double x = turretLocation.getX();
        double y = turretLocation.getY();
        double redX =  Constants.Hood.trenchRedX; 
        double blueX = Constants.Hood.trenchBlueX;
        double offset = Constants.Hood.trenchOffset;

        if (rob.isRed()){
            return (x < redX + offset && x > redX - offset) &&  (y < 1.37889 || y > 6.6842892) ; // should be 0.2 meter off trench for x and 0.1 for y // TODO: CONSTANTS
        } else {
            return (x < blueX + offset && x > blueX - offset) &&  (y < 1.37889 || y > 6.6842892); // should be 0.5 meter off trench for x and 0.1 for y
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Rotations", getRotations());
        SmartDashboard.putNumber("Hood Degrees", getDegrees());
        SmartDashboard.putNumber("Hood", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood Current", getCurrent());
        SmartDashboard.putBoolean("Will get decapitated", willGetDecapitated());

        // degrees = SmartDashboard.getNumber("Set Angles", 0);
        double future[] = rob.mmturret.aLookIntoTheFuture(swerve.getPoseMeters(), rob.mmturret.getDistanceToTarget(), rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue); 
        if (rob.mmturret.isInAllianceZone()){
            // alignHoodtoHub(rob.mmturret.getDistanceToTarget());
            alignHoodtoHub(future[0]); 
        } else {
            //alignHoodtoLob(rob.mmturret.getDistanceToTarget());
            alignHoodtoLob(future[0]); 
        }

        if (willGetDecapitated()){
            degrees = 0.5;
            setDegrees();
        }
    }
}