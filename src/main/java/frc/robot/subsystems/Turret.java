package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor = new TalonFX(Constants.CanIDs.turretMotorID, new CANBus("Jeff"));
    
    private PositionVoltage pv = new PositionVoltage(0).withSlot(0);
    private Slot0Configs slot0Configs;
   
    private final CommandSwerveDrivetrain swerve;
    private final RobotContainer rob;

    private double targetDegrees;
    private double distanceToTarget;

    public Turret(RobotContainer rob) {
        this.swerve = rob.drivetrain;
        this.rob = rob;

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
        turretMotor.setControl(new PositionVoltage(0));
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
        Pose2d currentPos = swerve.getPoseMeters();

        // Pose2d currentPos = rob.simulatedPosition;
        double xDistance = currentPos.getX() - targetPos.getX();
        double yDistance = currentPos.getY() - targetPos.getY();  
        setDistanceToTarget(Math.hypot(xDistance, yDistance)); // the distance to target # dr afsari core
        double aimAngle = Math.toDegrees(Math.atan(xDistance/yDistance));

        double offset = 90; // default idk

        //pretty sure q2 and q3 are for shooting, and it works. q1 and q4 for lobbing (i think)
        
        if(xDistance >=0 && yDistance >= 0) { // q1
            offset = 90; 
        }
        
        if(xDistance <=0 && yDistance >= 0) { // q2
            offset = 90; 
        }

        if(xDistance <= 0 && yDistance <= 0) { // q3
            offset = -90; 
        }   

        if(xDistance >= 0 && yDistance <= 0) { // q4
            offset = -90;
        }

        double degreesToTarget = offset - aimAngle - (currentPos.getRotation().getDegrees() - targetPos.getRotation().getDegrees());

        SmartDashboard.putNumber("turret degrees to target", degreesToTarget);
        SmartDashboard.putNumber("turret aimangle", aimAngle);
        SmartDashboard.putNumber("turret offset", offset);
        
        setDegrees(degreesToTarget); 
        // TODO: offset target by robot rotations: currentPos.getRotation().getDegrees() - targetPos.getRotation().getDegrees();
        // TODO: offset turret location based on robot position/rotations
        // TODO: offset turret position to actual position on robot
    }

    public void setDistanceToTarget(double dist) {
        distanceToTarget = dist;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public boolean canMove() {
        return Math.abs(getDegrees()) < 90;
    }

    @Override
    public void periodic() {
        // point turret at target
        // see: celtx 2022 ShooterSubsystem.java line 339
        pointAtTarget(Constants.targetPosition);

        if(!canMove()) {
            stop(); // TODO: spin turret to avoid EXPLOSION
        }

        SmartDashboard.putNumber("Turret Angle Degrees", getDegrees());
        SmartDashboard.putNumber("Turret Rotation", getRotations());
        SmartDashboard.putNumber("Turret Error Degrees", targetDegrees-getDegrees());
    }
}
