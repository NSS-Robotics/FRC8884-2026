package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterHood extends SubsystemBase {
    private final TalonFX hoodMotor = new TalonFX(Constants.CanIDs.shooterHoodMotorID);
    
    private PositionVoltage pv = new PositionVoltage(0).withSlot(0);
    private Slot0Configs slot0Configs;
    
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

        slot0Configs = new Slot0Configs();
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
        //this.zeroHood();
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
        // 0.45x^3 - 5.53x^2 + 22.04x - 19.18; subject to change, this one is just a test also passed certain meters it goes down
        //TODO: get actual curve
        // double angle = 0.45 * Math.pow(distance, 3) - 5.53 * Math.pow(distance, 2) + 22.04 * distance - 19.18;
        // double angle = 0.42 * Math.pow(distance, 3) - 4.72 * Math.pow(distance, 2) + 19.18 * distance - 18.21;
        // double angle = 0.10 * Math.pow(distance, 3) - 1.53 * Math.pow(distance, 2) + 10.36 * distance - 11.02;
        double angle = 0.06 * Math.pow(distance, 3) - 0.54 * Math.pow(distance, 2) + 5.33 * distance - 4.88;
        angle = Math.max(Math.min(Constants.Hood.maxHubHoodAlign, angle), 0);
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

    private void changeDegrees(double deg) {
        double rotations = (getDegrees() + deg) * Constants.Hood.gearRatio / 360;
        setRotations(rotations);
    }

    public double getDegrees() {
        return 360 * getRotations() / Constants.Hood.gearRatio;
        //return degrees;
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
        hoodMotor.setPosition(0);
        degrees = 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Rotations", getRotations());
        SmartDashboard.putNumber("Hood Degrees", getDegrees());
        SmartDashboard.putNumber("Hood", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood Current", getCurrent());

        degrees = SmartDashboard.getNumber("Set Angles", 0);
        // if (mmturret.isInAllianceZone()){
        //     alignHoodtoHub(mmturret.getDistanceToTarget());
        // }else{
        //     alignHoodtoLob(mmturret.getDistanceToTarget());
        // }
        double future[] = rob.mmturret.aLookIntoTheFuture(swerve.getPoseMeters(), rob.mmturret.getDistanceToTarget(), Constants.targetPositionBlue); // TODO: MAKE THE RED THING CHANGABLE

        alignHoodtoHub(future[0]); 
    }
}