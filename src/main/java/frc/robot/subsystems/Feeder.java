package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {
    private final TalonFX feederMotor = new TalonFX(Constants.CanIDs.feederMotorID, new CANBus("Jeff"));
    private final DutyCycleOut feedOut = new DutyCycleOut(0);
    private double targetSpeed = 0;
    private final double maxPower = 0.9;
    private final double minPower = 0.4;

    private boolean canMove = true;
    private final MotionMagicTurret mmturret;

    private VelocityVoltage feederVelocity = new VelocityVoltage(0).withSlot(0);

    public Feeder(RobotContainer rob) {
        this.mmturret = rob.mmturret;
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        currentLimitConfig.StatorCurrentLimit = Constants.Feeder.currentLimit;
        currentLimitConfig.StatorCurrentLimitEnable = true;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Feeder.kP;
        slot0Configs.kI = Constants.Feeder.kI;
        slot0Configs.kD = Constants.Feeder.kD;
        slot0Configs.kS = Constants.Feeder.kS;
        slot0Configs.kV = Constants.Feeder.kV;
        slot0Configs.kA = Constants.Feeder.kA;

        feederMotor.getConfigurator().apply(slot0Configs);
        feederMotor.getConfigurator().apply(currentLimitConfig);
        feederMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setSpeed(double speedRPM) {
        targetSpeed = speedRPM/60;
        feederMotor.setControl(feederVelocity.withVelocity(speedRPM / 60));
    }
    public void makeSpin(double power) {
        //spindexerMotor.set(power);
        //spindexerMotor.setControl(new DutyCycleOut(power));
        feederMotor.setControl(feedOut.withOutput(power));
    }

    public double getSpeed() {
        return feederMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        targetSpeed = 0;
        feederMotor.setControl(new VoltageOut(0));
    }

    public void canMove() {
        canMove = !(
            Math.abs(mmturret.targetDegrees-mmturret.getDegrees()) > Constants.Turret.tolerance
        );
    }

    @Override
    public void periodic() {
        // double ts = Math.abs(targetSpeed);
        // double speed = Math.abs(getSpeed());
        // double power = (ts - speed)/ts * maxPower;
        // power = Math.min(Math.max(power, minPower), maxPower);
        // if (speed > ts) {
        //     power = 0;
        // }
        // if (targetSpeed == 0){
        //     power =0;
        // }

        // if (targetSpeed < 0) {
        //     feederMotor.setControl(feedOut.withOutput(-power));
        // } else {
        //     feederMotor.setControl(feedOut.withOutput(power));
        // }

        SmartDashboard.putNumber("Feeder Speed", getSpeed());
        SmartDashboard.putBoolean("canMove", canMove);
        canMove();
    }
}
