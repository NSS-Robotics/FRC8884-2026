package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
    private final TalonFX spindexerMotor = new TalonFX(Constants.CanIDs.spindexerMotorID, new CANBus("Jeff"));
    private final DutyCycleOut spinOut = new DutyCycleOut(0);
    private final double maxPower = 0.6;
    private final double minPower = 0.1;
    public boolean isAutoShoot = false;
    // private final double maxPower = 0.8;
    private double targetSpeed = 0.0;

    public Spindexer() {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Spindexer.kP;
        slot0Configs.kI = Constants.Spindexer.kI;
        slot0Configs.kD = Constants.Spindexer.kD;
        slot0Configs.kS = Constants.Spindexer.kS;
        slot0Configs.kV = Constants.Spindexer.kV;
        slot0Configs.kA = Constants.Spindexer.kA;
        //spindexerMotor.getConfigurator().apply(slot0Configs);

        spindexerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double rotationsPerMinute) {
        targetSpeed = rotationsPerMinute/60;
        // spindexerMotor.setControl(new VelocityVoltage(rotationsPerMinute/60));
    }

    public double getSpeed() {
        return spindexerMotor.getVelocity().getValueAsDouble();
    }

    public void makeSpin(double power) {
        //spindexerMotor.set(power);
        //spindexerMotor.setControl(new DutyCycleOut(power));
        spindexerMotor.setControl(spinOut.withOutput(power));
    }

    public void stop() {
        spindexerMotor.setControl(new VoltageOut(0));
        targetSpeed = 0;
    }

    @Override
    public void periodic() {
        if (isAutoShoot){
            targetSpeed = -3000;
        }
        double ts = Math.abs(targetSpeed);
        double speed = Math.abs(getSpeed());
        double power = (ts - speed)/ts * maxPower;
        power = Math.min(Math.max(power, minPower), maxPower);
        if (speed > ts) {
            power = 0;
        }
        if (targetSpeed == 0){
            power = 0;
        }

        if (targetSpeed < 0) {
            spindexerMotor.setControl(spinOut.withOutput(-power));
        } else {
            spindexerMotor.setControl(spinOut.withOutput(power));
        }

        SmartDashboard.putNumber("Spindexer Speed", getSpeed());
    }
}
