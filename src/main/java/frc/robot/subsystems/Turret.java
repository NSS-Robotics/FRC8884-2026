package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor = new TalonFX(Constants.CanIDs.turretMotorID);
    
    private PositionVoltage pv = new PositionVoltage(0).withSlot(0);

    private Slot0Configs slot0Configs;
   
    // private final CommandSwerveDrivetrain swerve;

    private double targetDegrees;

    public Turret(RobotContainer rob) {
        // this.swerve = rob.drivetrain;

        TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
        // turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // turretMotorConfig.Feedback.FeedbackRemoteSensorID = Constants.CanIDs.turretEncoderID;
        CurrentLimitsConfigs turretMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
        turretMotorCurrentLimitsConfigs.StatorCurrentLimit = 15;
        turretMotorCurrentLimitsConfigs.StatorCurrentLimitEnable = true;

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
        turretMotor.getConfigurator().apply(turretMotorCurrentLimitsConfigs);

        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        turretMotor.setPosition(0);
    }

    public void setRotations(double rotations) {
        turretMotor.setControl(pv.withPosition(rotations));
    }

    public double getRotations() {
        return turretMotor.getPosition().getValueAsDouble();
    }


    public void stop() {
        turretMotor.setControl(new VoltageOut(0));
    }


    public void setDegrees(double degrees) {
        targetDegrees = degrees;
        double rotations = degrees * Constants.Turret.gearRatio / 360;
        SmartDashboard.putNumber("Turret Target Degrees", degrees);
        turretMotor.setControl(pv.withPosition(rotations));
    }

    public double getDegrees() {
        return 360 * getRotations() / Constants.Turret.gearRatio;
    }


    public void pointAtTarget(Pose2d targetPos) {
        // Pose2d currentPos = swerve.getPoseMeters();
         
        Pose2d currentPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        double distanceToTarget = Math.hypot(currentPos.getX() - targetPos.getX(), currentPos.getY() - targetPos.getY()); // the distance to target # dr afsari core
        double degreesToTarget = currentPos.getRotation().getDegrees() - targetPos.getRotation().getDegrees();
        setDegrees(degreesToTarget);
    }

    @Override
    public void periodic() {
        // point turret at target
        // see: celtx 2022 ShooterSubsystem.java line 339

        SmartDashboard.putNumber("Turret Angle Degrees", getDegrees());
        SmartDashboard.putNumber("Turret Rotation", getRotations());
        SmartDashboard.putNumber("Turret Error", targetDegrees-getDegrees());
    }
}
