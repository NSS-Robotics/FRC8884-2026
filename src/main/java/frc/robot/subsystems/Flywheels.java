package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
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

    private final RobotContainer rob;

    private double targetVelocity = 9000;

    public Flywheels(RobotContainer rob) {
        this.rob = rob;
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        currentLimitConfig.StatorCurrentLimit = Constants.Flywheel.currentLimit;
        currentLimitConfig.StatorCurrentLimitEnable = true;

        Slot0Configs leftPIDConfigs = new Slot0Configs();
        leftPIDConfigs.kP = Constants.Flywheel.kP;
        leftPIDConfigs.kI = Constants.Flywheel.kI;
        leftPIDConfigs.kD = Constants.Flywheel.kD;
        leftPIDConfigs.kS = Constants.Flywheel.kS;
        leftPIDConfigs.kV = Constants.Flywheel.kV;
        leftPIDConfigs.kA = Constants.Flywheel.kA;

        Slot0Configs rightPIDConfigs = new Slot0Configs();
        rightPIDConfigs.kP = Constants.Flywheel.kP;
        rightPIDConfigs.kI = Constants.Flywheel.kI;
        rightPIDConfigs.kD = Constants.Flywheel.kD;
        rightPIDConfigs.kS = Constants.Flywheel.kS;
        rightPIDConfigs.kV = Constants.Flywheel.kV;
        rightPIDConfigs.kA = Constants.Flywheel.kA;

        SmartDashboard.putNumber("Set RPM", 0);

        rightFlywheelMotor.getConfigurator().apply(rightConfig);
        leftFlywheelMotor.getConfigurator().apply(leftConfig);

        rightFlywheelMotor.getConfigurator().apply(currentLimitConfig);
        leftFlywheelMotor.getConfigurator().apply(currentLimitConfig);

        rightFlywheelMotor.getConfigurator().apply(rightPIDConfigs);
        leftFlywheelMotor.getConfigurator().apply(leftPIDConfigs);

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
        // return 51.61 * Math.pow(distance, 3) - 375.59 * Math.pow(distance, 2) + 1147.77 * distance + 533.44; // TODO: GET CURVE
        return Math.max(Math.min(29.54 * Math.pow(distance, 3) - 299.81 * Math.pow(distance, 2) + 1088.56 * distance + 527.65, 3000), 0);
    }

    public void rpmToLob(double distance){

    }

    public void rpmToRest(){
        // TODO: resting set resting rpm
        setVelocity(500);
    }

    // public void makeThemSpin(double power) {
    //     flywheelMotor.set(power);
    //     followerFlywheelMotor.set(power);
    // }



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