package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    private final TalonFX feederMotor = new TalonFX(Constants.CanIDs.feederMotorID);

    private VelocityVoltage feederVelocity = new VelocityVoltage(0).withSlot(0);

    public Feeder() {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Feeder.kP;
        slot0Configs.kI = Constants.Feeder.kI;
        slot0Configs.kD = Constants.Feeder.kD;
        slot0Configs.kS = Constants.Feeder.kS;
        slot0Configs.kV = Constants.Feeder.kV;
        slot0Configs.kA = Constants.Feeder.kA;
        feederMotor.getConfigurator().apply(slot0Configs);
    }

    public void setSpeed(double speedRPM) {
        feederMotor.setControl(feederVelocity.withVelocity(speedRPM / 60));
    }

    public double getSpeed() {
        return feederMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        feederMotor.setControl(new VoltageOut(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Speed", getSpeed());
    }
}
