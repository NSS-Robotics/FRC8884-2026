package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        currentLimitConfig.StatorCurrentLimit = Constants.Flywheel.currentLimit;
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
        SmartDashboard.putNumber("fly s", 0);
        SmartDashboard.putNumber("fly v", 0);

        operatorController = rob.operatorController.getHID();
        this.rob = rob;
    }

    public void setVelocity(double rpm) {
        targetVelocity = (rpm)/60; // +25

        rightFlywheelMotor.setControl(new VelocityVoltage(-rpm/60));
        leftFlywheelMotor.setControl(new VelocityVoltage(rpm/60));
    }

    public double getVelocity() {
        return leftFlywheelMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        // targetVelocity = 500;
        // VoltageOut voltageOut = new VoltageOut(0);
        // rightFlywheelMotor.setControl(voltageOut);
        // leftFlywheelMotor.setControl(voltageOut);
        rpmToRest();
        // :)
    }

    public double rpmToHub(double distance) {
        double d = Math.min(distance, 9);

        // NEW CALI (as of March 17)
        if (d <= 5) {
            double ans = 7.19 * Math.pow(d, 3) - 63.18 * Math.pow(d, 2) + 329.55 * d + 927.81;

            return Math.max(Math.min(ans, 3700), 0); // Clamp RPM to between 0 and 3700
        } else {
            return Math.max(Math.min(-33.41 * Math.pow(d, 3) + 692.5 * Math.pow(d, 2) - 4440.80 * d + 10990.35, 3500), 0);
        }

        // BAD CALI (as of March 16 - turned out to be bad)
        // if (d <= 5) {
        //     double ans = -3.76 * Math.pow(d, 3) + 45.71 * Math.pow(d, 2) + 22.76 * d + 1178.77;
        //     return Math.max(Math.min(ans, 3700), 0); // Clamp RPM to between 0 and 3700
        // } else {
        //     return Math.max(Math.min(-33.41 * Math.pow(d, 3) + 692.5 * Math.pow(d, 2) - 4440.80 * d + 10990.35, 3500), 0);
        // }

        // OLD CALI
        // d-=0.135;
        // if (d < 5.1){
        //     // return Math.max(Math.min(-23.63 * Math.pow(d, 3) + 236.53 * Math.pow(d, 2) - 493.68 * d + 1860.02, 3000), 0);
        //     double ans = 2.52 * Math.pow(d, 3) - 23.56 * Math.pow(d, 2) + 244.19 * d + 930.74;
        //     if (d > 4.5){
        //         d -= 0.1;
        //     }
        //     return Math.max(Math.min(ans, 3500), 0); 
        // } else  {
        //     return Math.max(Math.min(-33.41 * Math.pow(d, 3) + 692.5 * Math.pow(d, 2) - 4440.80 * d + 10990.35, 3500), 0); 
        // }
        //     // return Math.max(Math.min(-19.11 * Math.pow(d, 3) + 436.3 * Math.pow(d, 2) - 2814.13 * d + 7738.87, 4000), 0);
    }

    public double rpmToLob(double distance){
        return 100*distance + 1100;
    }

    public void rpmToRest() {
        setVelocity(1000);
    }

    @Override
    public void periodic() {
        boolean velocityReached = targetVelocity <= getVelocity() - 3;
        SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
        SmartDashboard.putBoolean("Flywheel Velocity Reached", velocityReached);
        
        if (velocityReached && rob.isShooting && !DriverStation.isAutonomous()) {
            operatorController.setRumble(RumbleType.kBothRumble, 0.5);
        }

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

        // SmartDashboard.putNumber("FLYWHEEL RPM", )

        // double p = SmartDashboard.getNumber("fly p", 0);
        // double s = SmartDashboard.getNumber("fly s", 0);
        // double v = SmartDashboard.getNumber("fly v", 0);
        
        // pidConfigs.kP = p;
        // pidConfigs.kS = s;
        // pidConfigs.kV = v;

        // boolean isChanged = false;

        // if(p != pidConfigs.kP){
        // pidConfigs.kP = p;
        // isChanged = true; 
        // }

        // if(s != pidConfigs.kS){
        // pidConfigs.kS = s;
        // isChanged = true;
        // }

        // if(v != pidConfigs.kV){
        // pidConfigs.kV = v;
        // isChanged = true;
        // }

        // if(isChanged){
        //     leftFlywheelMotor.getConfigurator().apply(pidConfigs);
        //     rightFlywheelMotor.getConfigurator().apply(pidConfigs);
        // }
        // leftFlywheelMotor.getConfigurator().apply(pidConfigs);
        // rightFlywheelMotor.getConfigurator().apply(pidConfigs);
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }
}