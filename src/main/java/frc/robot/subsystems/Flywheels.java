package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheels extends SubsystemBase {
    private final TalonFX rightFlywheelMotor = new TalonFX(Constants.CanIDs.rightFlywheelMotorID);
    private final TalonFX leftFlywheelMotor = new TalonFX(Constants.CanIDs.leftFlywheelMotorID);

    private VelocityVoltage flywheelVoltage = new VelocityVoltage(0).withSlot(0);

    public Flywheels() {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs leftPIDConfigs = new Slot0Configs();
        leftPIDConfigs.kP = Constants.Flywheel.kP;
        leftPIDConfigs.kI = Constants.Flywheel.kI;
        leftPIDConfigs.kD = Constants.Flywheel.kD;
        leftPIDConfigs.kS = Constants.Flywheel.kS;
        leftPIDConfigs.kV = Constants.Flywheel.kV;
        leftPIDConfigs.kA = Constants.Flywheel.kA;

        Slot0Configs rightPIDConfigs = new Slot0Configs();
        rightPIDConfigs.kP = Constants.Flywheel.kP;
        rightPIDConfigs.kI = Constants.Flywheel.kI;
        rightPIDConfigs.kD = Constants.Flywheel.kD;
        rightPIDConfigs.kS = Constants.Flywheel.kS;
        rightPIDConfigs.kV = Constants.Flywheel.kV;
        rightPIDConfigs.kA = Constants.Flywheel.kA;

        rightFlywheelMotor.getConfigurator().apply(rightConfig);
        leftFlywheelMotor.getConfigurator().apply(leftConfig);

        rightFlywheelMotor.getConfigurator().apply(rightPIDConfigs);
        leftFlywheelMotor.getConfigurator().apply(leftPIDConfigs);

        rightFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        leftFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);

    }

    public void setVelocity(double targetSpeedRPM) {
        rightFlywheelMotor.setControl(new VelocityVoltage(targetSpeedRPM/60));
        leftFlywheelMotor.setControl(new VelocityVoltage(-targetSpeedRPM/60));
    }

    public double getVelocity() {
        return rightFlywheelMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        VoltageOut voltageOut = new VoltageOut(0);
        rightFlywheelMotor.setControl(voltageOut);
        leftFlywheelMotor.setControl(voltageOut);
    }

    // public void makeThemSpin(double power) {
    //     flywheelMotor.set(power);
    //     followerFlywheelMotor.set(power);
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }
}