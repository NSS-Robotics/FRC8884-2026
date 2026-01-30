package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
        hoodMotor.setPosition(0);
    }

    public void setRotations(double rotations) {
        hoodMotor.setControl(pv.withPosition(rotations));
    }

    public double getRotations() {
        return hoodMotor.getPosition().getValueAsDouble();
    }


    public void stop() {
        hoodMotor.setControl(new VoltageOut(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Rotations", getRotations());
    }
}
