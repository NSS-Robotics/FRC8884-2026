package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Constants.CanIDs.intakePivotMotorID);
  private final CANcoder encoder = new CANcoder(Constants.CanIDs.intakePivotEncoderID);

  private PositionVoltage pv = new PositionVoltage(0).withSlot(0);

  // Motor configuration
  private TalonFXConfiguration motorConfig;
  private Slot0Configs slot0Configs;
  
  public IntakePivot(RobotContainer rob) {
    slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Intake.kP;
    slot0Configs.kI = Constants.Intake.kI;
    slot0Configs.kD = Constants.Intake.kD;
    slot0Configs.kS = Constants.Intake.kS;
    slot0Configs.kV = Constants.Intake.kV;
    slot0Configs.kA = Constants.Intake.kA;

    motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    motor.getConfigurator().apply(motorConfig);
    motor.getConfigurator().apply(slot0Configs);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setPosition(double position) {
    motor.setControl(pv.withVelocity(position));
  }

  public double getPosition() {
    return encoder.getPosition().getValueAsDouble();
  }

  public void stop() {
    motor.setControl(new VoltageOut(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Pivot Position", getPosition());
    SmartDashboard.putNumber("Intake Pivot Current", motor.getStatorCurrent().getValueAsDouble());
  }
}