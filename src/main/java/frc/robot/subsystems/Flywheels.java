package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Flywheels extends SubsystemBase {
    private final TalonFX rightFlywheelMotor = new TalonFX(Constants.CanIDs.rightFlywheelMotorID);
    private final TalonFX leftFlywheelMotor = new TalonFX(Constants.CanIDs.leftFlywheelMotorID);

    private final XboxController operatorController;
    private final RobotContainer rob;

    private double targetVelocity = 0;
    // private final double maxPower = 1.0;
    // private final double minPower = 0.1;

    // private final DutyCycleOut flywheelOutL = new DutyCycleOut(0);
    // private final DutyCycleOut flywheelOutR = new DutyCycleOut(0);

    Slot0Configs pidConfigs = new Slot0Configs();

    public Flywheels(RobotContainer rob) {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leftConfig.TorqueCurrent.withPeakForwardTorqueCurrent(100).withPeakReverseTorqueCurrent(-20);
        rightConfig.TorqueCurrent.withPeakForwardTorqueCurrent(100).withPeakReverseTorqueCurrent(-20);

        // leftConfig.TorqueCurrent.withPeakForwardTorqueCurrent(40).withPeakReverseTorqueCurrent(-40);
        // rightConfig.TorqueCurrent.withPeakForwardTorqueCurrent(40).withPeakReverseTorqueCurrent(-40);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        currentLimitConfig.StatorCurrentLimit = Constants.Flywheel.currentLimit;
        currentLimitConfig.SupplyCurrentLimit = Constants.Flywheel.supplyLimit;
        currentLimitConfig.SupplyCurrentLowerLimit = Constants.Flywheel.supplyLimit;
        currentLimitConfig.StatorCurrentLimitEnable = true;

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

        SmartDashboard.putNumber("fly p", 0);
        SmartDashboard.putNumber("fly i", 0);
        SmartDashboard.putNumber("fly d", 0);
        SmartDashboard.putNumber("fly s", 0);
        SmartDashboard.putNumber("fly v", 0);
        SmartDashboard.putNumber("fly a", 0);
        
        Follower rightFollower = new Follower(leftFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed);
        rightFlywheelMotor.setControl(rightFollower);

        operatorController = rob.operatorController.getHID();
        this.rob = rob;
    }

    public void setVelocity(double rpm) {
        targetVelocity = (rpm)/60;

        // rightFlywheelMotor.setControl(new VelocityVoltage(-rpm/60));
        // leftFlywheelMotor.setControl(new VelocityVoltage(rpm/60));

        //rightFlywheelMotor.setControl(new VelocityTorqueCurrentFOC(-rpm/60).withSlot(0));
        leftFlywheelMotor.setControl(new VelocityTorqueCurrentFOC(rpm/60).withSlot(0));
    }

    public double getVelocity() {
        return leftFlywheelMotor.getVelocity().getValueAsDouble();
    }

    public boolean isAtTargetVelocity(double toleranceRPS) {
        double velocity = getVelocity();

        return targetVelocity >= velocity - toleranceRPS;
    }

    public void stop() {
        setVelocity(0);
    }

    public double rpmToHub(double distance) {
        double d = Math.min(distance, 9);

        if (d > 3.0) {
            d -= 0.1;
        }

        double ans = -1.83 * Math.pow(d, 4) + 34.3 * Math.pow(d, 3) - 231 * Math.pow(d, 2) + 859 * d + 1535;
        return Math.max(Math.min(ans, 3700), 0);
    }

    public double rpmToLob(double distance){
        return (200*distance + 2600) *
            (rob.turret.isInOtherAllianceZone() ? 3 : 1) +
            (rob.turret.isInOtherAllianceZone() ? 1000 : 0);
    }

    public void rpmToRest() {
        setVelocity(0);
    }

    @Override
    public void periodic() {
        boolean velocityReached = targetVelocity <= getVelocity() - 3;
        SmartDashboard.putNumber("Flywheel Velocity", getVelocity()*60);
        SmartDashboard.putNumber("Flywheel Target Velocity", targetVelocity*60);

        SmartDashboard.putBoolean("Flywheel Velocity Reached", velocityReached);
        
        // if (velocityReached && rob.isShooting && !DriverStation.isAutonomous()) {
        //     operatorController.setRumble(RumbleType.kBothRumble, 0.5);
        // }

        // if (turret.isInAllianceZone()){
        //     rpmToHub(rob.turret.getDistanceToTarget());
        // } // check if its gonna lob
        // else{
        //     rpmToRest();
        // }

        // double ts = Math.abs(targetVelocity);
        // double speed = Math.abs(getVelocity());
        // double power = (ts - speed)/ts;

        // power = Math.min(Math.max(power, minPower), maxPower);
        // if (targetVelocity == 0){
        //     power =0;
        // }

        // if (targetVelocity < 0) {
        //     leftFlywheelMotor.setControl(flywheelOutL.withOutput(-power));
        //     rightFlywheelMotor.setControl(flywheelOutR.withOutput(power));
        // } else {
        //     leftFlywheelMotor.setControl(flywheelOutL.withOutput(power));
        //     rightFlywheelMotor.setControl(flywheelOutR.withOutput(-power));
        // }

        // double p = SmartDashboard.getNumber("fly p", 0);
        // double i = SmartDashboard.getNumber("fly i", 0);
        // double d = SmartDashboard.getNumber("fly d", 0);
        // double s = SmartDashboard.getNumber("fly s", 0);
        // double v = SmartDashboard.getNumber("fly v", 0);
        // double a = SmartDashboard.getNumber("fly a", 0);

        // boolean isChanged = (
        //     p != pidConfigs.kP ||
        //     i != pidConfigs.kI ||
        //     d != pidConfigs.kD ||
        //     s != pidConfigs.kS ||
        //     v != pidConfigs.kV ||
        //     a != pidConfigs.kA);
        
        // if (isChanged) {
        //     pidConfigs.kP = p;
        //     pidConfigs.kI = i;
        //     pidConfigs.kD = d;
        //     pidConfigs.kS = s;
        //     pidConfigs.kV = v;
        //     pidConfigs.kA = a;
        //     leftFlywheelMotor.getConfigurator().apply(pidConfigs);
        //     rightFlywheelMotor.getConfigurator().apply(pidConfigs);
        // }
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }
}