package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheels extends SubsystemBase {
    private final TalonFX flywheelMotor = new TalonFX(Constants.CanIDs.flywheelMotorID);

    private VelocityVoltage flywheelVoltage;

    public Flywheels() {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Flywheel.kP;
        slot0Configs.kI = Constants.Flywheel.kI;
        slot0Configs.kD = Constants.Flywheel.kD;
        slot0Configs.kS = Constants.Flywheel.kS;
        slot0Configs.kV = Constants.Flywheel.kV;
        slot0Configs.kA = Constants.Flywheel.kA;
        flywheelMotor.getConfigurator().apply(slot0Configs);
    }

    public void setVelocity(double targetSpeedRPM) {
        flywheelVoltage = new VelocityVoltage(targetSpeedRPM / 60);
        flywheelMotor.setControl(flywheelVoltage);
    }

    public double getVelocity() {
        return flywheelMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        VoltageOut voltageOut = new VoltageOut(0);
        flywheelMotor.setControl(voltageOut);
    }
}
