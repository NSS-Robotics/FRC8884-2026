package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Constants.CanIDs.intakePivotMotorID, new CANBus("Jeff"));

  private PositionVoltage pv = new PositionVoltage(0).withSlot(0);

  // Motor configuration
  private Slot0Configs slot0Configs;

  
  public IntakePivot(RobotContainer rob) {
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimit = Constants.IntakePivot.currentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.IntakePivot.kP;
    slot0Configs.kI = Constants.IntakePivot.kI;
    slot0Configs.kD = Constants.IntakePivot.kD;
    slot0Configs.kS = Constants.IntakePivot.kS;
    slot0Configs.kV = Constants.IntakePivot.kV;
    slot0Configs.kA = Constants.IntakePivot.kA;

    // motorConfig = new TalonFXConfiguration();
    // motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    // motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    // motor.getConfigurator().apply(motorConfig);
    motor.getConfigurator().apply(slot0Configs);
    motor.getConfigurator().apply(currentLimitsConfigs);
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.setPosition(0);
    
  }

  public void setPosition(double position) {
    SmartDashboard.putNumber("target", position);
    motor.setControl(pv.withPosition(position));
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public boolean isIntakeUp(){
    return getPosition() < 1;
  }

  public void stop() {
    motor.setControl(new VoltageOut(0));
  }

      public double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void zeroIntake() {
        while (getCurrent() < 15) {
            setPosition(1);
        }
        this.stop();

        // tell the hood that this is the correct zero position
        motor.setPosition(15.5);

    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Pivot Position", getPosition());
    SmartDashboard.putNumber("Intake Pivot Current", motor.getStatorCurrent().getValueAsDouble());
  }
}
