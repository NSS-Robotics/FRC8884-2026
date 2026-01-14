package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO: TEST PROTOTYPE CODE

public class Flywheels extends SubsystemBase {
    private final TalonFX flywheelMotor = new TalonFX(Constants.CanIDs.flywheelMotorID);
    private final TalonFX followerFlywheelMotor = new TalonFX(Constants.CanIDs.followerFlyWheelMotorID);

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

        flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        
        followerFlywheelMotor.setControl(new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Aligned));
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
    }
}

// TODO: motors shouldn't spin in same direction...