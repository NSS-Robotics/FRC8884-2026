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
    private final double minPower = 0.6;

    private boolean canMove = true;
    private final MotionMagicTurret mmturret;

    private Slot0Configs pidConfigs = new Slot0Configs();

    private VelocityVoltage feederVelocity = new VelocityVoltage(0).withSlot(0);

    public Feeder(RobotContainer rob) {
        this.mmturret = rob.mmturret;
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        currentLimitConfig.StatorCurrentLimit = Constants.Feeder.currentLimit;
        currentLimitConfig.StatorCurrentLimitEnable = true;

        pidConfigs.kP = Constants.Feeder.kP;
        pidConfigs.kI = Constants.Feeder.kI;
        pidConfigs.kD = Constants.Feeder.kD;
        pidConfigs.kS = Constants.Feeder.kS;
        pidConfigs.kV = Constants.Feeder.kV;
        pidConfigs.kA = Constants.Feeder.kA;

        feederMotor.getConfigurator().apply(pidConfigs);
        feederMotor.getConfigurator().apply(currentLimitConfig);
        feederMotor.setNeutralMode(NeutralModeValue.Coast);

        SmartDashboard.putNumber("feed p", 0);
        SmartDashboard.putNumber("feed s", 0);
        SmartDashboard.putNumber("feed v", 0);
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
        //     power = 0;
        // }

        // if (targetSpeed < 0) {
        //     feederMotor.setControl(feedOut.withOutput(-power));
        // } else {
        //     feederMotor.setControl(feedOut.withOutput(power));
        // }

        SmartDashboard.putNumber("Feeder Speed", getSpeed());
        SmartDashboard.putBoolean("canMove", canMove);
        canMove();

        double p = SmartDashboard.getNumber("feed p", 0);
        double s = SmartDashboard.getNumber("feed s", 0);
        double v = SmartDashboard.getNumber("feed v", 0);

        boolean isChanged = (
            p != pidConfigs.kP ||
            s != pidConfigs.kS ||
            v != pidConfigs.kV
        );

        if(isChanged){
            pidConfigs.kP = p;
            pidConfigs.kS = s;
            pidConfigs.kV = v;
            // feederMotor.getConfigurator().apply(pidConfigs);
        }

    }
}
