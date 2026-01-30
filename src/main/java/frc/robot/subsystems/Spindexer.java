package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
    private final TalonFX spindexerMotor = new TalonFX(Constants.CanIDs.spindexerMotorID, new CANBus("Jeff"));
    private VelocityVoltage spindexerVelocity = new VelocityVoltage(0).withSlot(0);

    public Spindexer() {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Spindexer.kP;
        slot0Configs.kI = Constants.Spindexer.kI;
        slot0Configs.kD = Constants.Spindexer.kD;
        slot0Configs.kS = Constants.Spindexer.kS;
        slot0Configs.kV = Constants.Spindexer.kV;
        slot0Configs.kA = Constants.Spindexer.kA;
        spindexerMotor.getConfigurator().apply(slot0Configs);

        spindexerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double rotationsPerMinute) {
        spindexerMotor.setControl(new VelocityVoltage(rotationsPerMinute/60));
    }

    public double getSpeed() {
        return spindexerMotor.getVelocity().getValueAsDouble();
    }

    public void makeSpin(double power) {
        spindexerMotor.set(power);
    }

    public void stop() {
        spindexerMotor.setControl(new VoltageOut(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindexer Speed", getSpeed());
    }
}
