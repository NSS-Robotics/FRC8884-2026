package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO: TEST PROTOTYPE CODE

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor = new TalonFX(Constants.CanIDs.turretMotorID);
    
    private PositionVoltage pv = new PositionVoltage(0).withSlot(0);

    private Slot0Configs slot0Configs;

    public Turret() {
        TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
        // turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // turretMotorConfig.Feedback.FeedbackRemoteSensorID = Constants.CanIDs.turretEncoderID;

        slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Turret.kP;
        slot0Configs.kI = Constants.Turret.kI;
        slot0Configs.kD = Constants.Turret.kD;
        slot0Configs.kS = Constants.Turret.kS;
        slot0Configs.kV = Constants.Turret.kV;
        slot0Configs.kA = Constants.Turret.kA;

        turretMotor.clearStickyFaults();
        turretMotor.getConfigurator().apply(slot0Configs);
        // turretMotor.getConfigurator().apply(turretMotorConfig);

        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        turretMotor.setPosition(0);
    }

    public void setRotations(double rotations) {
        turretMotor.setControl(pv.withPosition(rotations));
    }

    // public void increaseAngle(double degrees) {}

    public double getRotations() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    public void stop() {
        turretMotor.setControl(new VoltageOut(0));
    }

    private double getCurrentPositionAsDegrees() {
        return 360*getRotations() / Constants.Turret.gearRatio;
    }

    @Override
    public void periodic() {
        // point turret at target
        // see: celtx 2022 ShooterSubsystem.java line 339

        SmartDashboard.putNumber("Turret Angle Degrees", getCurrentPositionAsDegrees());
        SmartDashboard.putNumber("Turret Rotation", getRotations());
    }
}
