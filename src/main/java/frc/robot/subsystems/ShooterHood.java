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

public class ShooterHood extends SubsystemBase {
    private final TalonFX hoodMotor = new TalonFX(Constants.CanIDs.shooterHoodMotorID, new CANBus("Jeff"));
    
    private PositionVoltage pv = new PositionVoltage(0).withSlot(0);
    private Slot0Configs slot0Configs;

    private double degrees = 0;

    public ShooterHood() {
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
        this.zeroHood();
    }

    public void setRotations(double rotations) {
        hoodMotor.setControl(pv.withPosition(rotations));
    }

    public double getRotations() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public void setDegrees() {
        degrees = Math.max(Math.min(Constants.Hood.maxHoodAngle, degrees), 0);

        double rotations = degrees * Constants.Hood.gearRatio / 360;
        setRotations(rotations);
    }

    private void changeDegrees(double deg) {
        double rotations = (getDegrees() + deg) * Constants.Hood.gearRatio / 360;
        setRotations(rotations);
    }

    public double getDegrees(){
        return 360 * getRotations() / Constants.Hood.gearRatio;
    }
    
    public double getCurrent() {
        return hoodMotor.getStatorCurrent().getValueAsDouble();
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
    }
}
