package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
    private final TalonFX spindexerMotor = new TalonFX(Constants.CanIDs.spindexerMotorID);
    private VelocityVoltage spindexerVelocity = new VelocityVoltage(0).withSlot(0);

    public void setSpeed(double rotationsPerMinute) {
        spindexerMotor.setControl(spindexerVelocity.withVelocity(rotationsPerMinute / 60));
    }

    public double getSpeed() {
        return spindexerMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        spindexerMotor.setControl(new VoltageOut(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindexer Speed", getSpeed());
    }
}
