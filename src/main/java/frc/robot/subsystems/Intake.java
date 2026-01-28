package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private static TalonFX intakeMotor = new TalonFX(
        Constants.CanIDs.intakeMotorID
    );

    private static Slot0Configs intakeSlot0Configs = new Slot0Configs();

    public Intake() {
        intakeSlot0Configs.kP = Constants.Intake.kP;
        intakeSlot0Configs.kI = Constants.Intake.kI;
        intakeSlot0Configs.kD = Constants.Intake.kD;
        intakeSlot0Configs.kS = Constants.Intake.kS;
        intakeSlot0Configs.kV = Constants.Intake.kV;
        intakeSlot0Configs.kA = Constants.Intake.kA;

        intakeMotor.getConfigurator().apply(intakeSlot0Configs);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setVelocity(double velocity) {
        intakeMotor.setControl(new VelocityVoltage(velocity / 60));
    }

    public double getVelocity() {
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    public void stopIntake() {
        intakeMotor.setControl(new VoltageOut(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Velocity", getVelocity());
    }
}