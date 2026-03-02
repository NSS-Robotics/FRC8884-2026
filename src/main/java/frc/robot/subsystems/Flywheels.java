package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Flywheels extends SubsystemBase {
    private final TalonFX rightFlywheelMotor = new TalonFX(Constants.CanIDs.rightFlywheelMotorID);
    private final TalonFX leftFlywheelMotor = new TalonFX(Constants.CanIDs.leftFlywheelMotorID);

    private double targetVelocity = 9000;

    public Flywheels(RobotContainer rob) {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        currentLimitConfig.StatorCurrentLimit = Constants.Flywheel.currentLimit;
        currentLimitConfig.StatorCurrentLimitEnable = true;

        Slot0Configs pidConfigs = new Slot0Configs();
        pidConfigs.kP = Constants.Flywheel.kP;
        pidConfigs.kI = Constants.Flywheel.kI;
        pidConfigs.kD = Constants.Flywheel.kD;
        pidConfigs.kS = Constants.Flywheel.kS;
        pidConfigs.kV = Constants.Flywheel.kV;
        pidConfigs.kA = Constants.Flywheel.kA;

        SmartDashboard.putNumber("Set RPM", 0);

        rightFlywheelMotor.getConfigurator().apply(rightConfig);
        leftFlywheelMotor.getConfigurator().apply(leftConfig);

        rightFlywheelMotor.getConfigurator().apply(currentLimitConfig);
        leftFlywheelMotor.getConfigurator().apply(currentLimitConfig);

        rightFlywheelMotor.getConfigurator().apply(pidConfigs);
        leftFlywheelMotor.getConfigurator().apply(pidConfigs);

        rightFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        leftFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setVelocity(double rpm) {
        targetVelocity = rpm;

        rightFlywheelMotor.setControl(new VelocityVoltage(-rpm/60));
        leftFlywheelMotor.setControl(new VelocityVoltage(rpm/60));
    }

    public double getVelocity() {
        return leftFlywheelMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        VoltageOut voltageOut = new VoltageOut(0);
        rightFlywheelMotor.setControl(voltageOut);
        leftFlywheelMotor.setControl(voltageOut);
    }

    public double rpmToHub(double distance) {
        if (distance < 4.49){
            return Math.max(Math.min(-23.63 * Math.pow(distance, 3) + 236.53 * Math.pow(distance, 2) - 493.68 * distance + 1860.02, 2500 ), 0);
        } else {
            return Math.max(Math.min(357.14 * Math.pow(distance, 3) - 5514.73 * Math.pow(distance, 2) + 28472.27 * distance - 46897.74, 2500 ), 0);
        }
    }

    public void rpmToLob(double distance){

    }

    public void rpmToRest(){
        // TODO: resting set resting rpm
        setVelocity(500);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
        SmartDashboard.putBoolean("Flywheel Velocity Reached", targetVelocity <= getVelocity() - 3);
        // if (mmturret.isInAllianceZone()){
        //     rpmToHub(rob.mmturret.getDistanceToTarget());
        // } // check if its gonna lob
        // else{
        //     rpmToRest();
        // }
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }
}