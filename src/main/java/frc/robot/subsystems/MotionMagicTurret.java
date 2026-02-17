package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MotionMagicTurret extends SubsystemBase {
    private final TalonFX turretMotor = new TalonFX(Constants.CanIDs.turretMotorID, new CANBus("Jeff"));
    private final CANcoder turretEncoder = new CANcoder(Constants.CanIDs.turretEncoderID, new CANBus("Jeff"));
    
    private MotionMagicDutyCycle mmdc = new MotionMagicDutyCycle(0).withSlot(0);
   
    private final RobotContainer rob;
    private final CommandSwerveDrivetrain swerve;
    private final IntakePivot intakePivot;

    private double targetDegrees;
    private double distanceToTarget;

    private final Translation2d offsetFromRobotCenter;

    public Pose2d turretLocation = new Pose2d();

    public MotionMagicTurret(RobotContainer rob, Translation2d translation) {
        this.intakePivot = rob.intakePivot;
        this.swerve = rob.drivetrain;
        this.rob = rob;
        this.offsetFromRobotCenter = translation;

        TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
        turretMotorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        CurrentLimitsConfigs turretMotorCurrentLimitsConfigs = turretMotorConfig.CurrentLimits;
        turretMotorCurrentLimitsConfigs.StatorCurrentLimit = 20;
        turretMotorCurrentLimitsConfigs.StatorCurrentLimitEnable = true;

        Slot0Configs slot0Configs = turretMotorConfig.Slot0;
        slot0Configs.kP = Constants.Turret.kP;
        slot0Configs.kI = Constants.Turret.kI;
        slot0Configs.kD = Constants.Turret.kD;
        slot0Configs.kS = Constants.Turret.kS;
        slot0Configs.kV = Constants.Turret.kV;
        slot0Configs.kA = Constants.Turret.kA;

        MotionMagicConfigs motionMagicConfigs = turretMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 20; // maximum for an x60 is 100rps
        motionMagicConfigs.MotionMagicAcceleration = 20*3; // 0.5s to reach, *5 to take 0.2 sec, allegedy
        // motionMagicConfigs.MotionMagicJerk = 25*2*10; // *10 than acceleration, optional, according to some guy on chief delphi

        SmartDashboard.putBoolean("Target Red Hub", true);

        turretMotor.clearStickyFaults();
        turretMotor.getConfigurator().apply(turretMotorConfig);

        turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setRotations(double rotations) {
        turretMotor.setControl(mmdc.withPosition(rotations));
        SmartDashboard.putNumber("MM Turret target rotation", rotations);
    }

    public double getRotations() {
        return turretMotor.getPosition().getValueAsDouble();
    }
    
    public void stop() {
        turretMotor.setControl(new VoltageOut(0));
    }

    public void setDegrees(double degrees) {
        targetDegrees = degrees;
        double rotations = degrees * Constants.Turret.gearRatioWithAbsoluteEncoder / 360; 

        SmartDashboard.putNumber("MM Turret Target Degrees", degrees);

        setRotations(rotations);
    }

    public double getDegrees() {
        return 360 * getRotations() / Constants.Turret.gearRatioWithAbsoluteEncoder;
    }

    private double wrapAngleDegrees(double angle){
        if (angle < -380.0) {
            angle += 360.0;
        } else if (angle >= 30.0) {
            angle -= 360.0;
        }
        return angle;
    }

    private Pose2d calculateTurretOffset(double xOffset, double yOffset, double robotRadians){
        // x and y axis inverted
        
        double d = Math.hypot(xOffset, yOffset);    
        double theta = Math.atan2(xOffset, yOffset);
        double alpha = theta + robotRadians - Math.PI/2; 
        double x = d * Math.cos(alpha);
        double y = d * Math.sin(alpha);

        return new Pose2d(x, y, null);
    }

    public void pointAtTarget(Pose2d targetPos) { 
        Pose2d currentPos = swerve.getPoseMeters();
        
        double robotRadians = currentPos.getRotation().getRadians(); 
        Pose2d turretOffset = calculateTurretOffset(offsetFromRobotCenter.getX(), offsetFromRobotCenter.getY(), robotRadians);

        turretLocation = new Pose2d(currentPos.getX() + turretOffset.getX(), currentPos.getY() + turretOffset.getY(), Rotation2d.fromDegrees(0));

        double xDistance = currentPos.getX() + turretOffset.getX() - targetPos.getX();
        double yDistance = currentPos.getY() + turretOffset.getY() - targetPos.getY();  

        setDistanceToTarget(Math.hypot(xDistance, yDistance));
        double aimAngle = Math.toDegrees(Math.atan(xDistance/yDistance));

        double offset = 90;
        
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

        double degreesToTarget = wrapAngleDegrees(offset - aimAngle - (currentPos.getRotation().getDegrees() - targetPos.getRotation().getDegrees()));
        SmartDashboard.putNumber("MM Turret degrees to target", degreesToTarget);
        SmartDashboard.putNumber("MM Turret Offset x", turretOffset.getX());
        SmartDashboard.putNumber("MM Turret Offset y", turretOffset.getY());
        
        setDegrees(degreesToTarget);
    }

    public void setDistanceToTarget(double dist) {
        distanceToTarget = dist;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public boolean canMove(){
        return !intakePivot.isIntakeUp();
    }

    public boolean isInAllianceZone() {
        if (rob.isRed()) {
            return swerve.getPoseMeters().getX() < 4;
        }

        return swerve.getPoseMeters().getX() > 12.5;
    }

    @Override
    public void periodic() {
        // point turret at target
        // see: celtx 2022 ShooterSubsystem.java line 339

        if (isInAllianceZone()) { 
            pointAtTarget(rob.lobTarget);
        } else {      
            if (SmartDashboard.getBoolean("Target Red Hub", true)) {
                pointAtTarget(Constants.targetPositionRed);
            } else {
                pointAtTarget(Constants.targetPositionBlue);
            }
        }

        if(!canMove()){
            stop();
        }

        SmartDashboard.putNumber("MM Turret Angle Degrees", getDegrees());
        SmartDashboard.putNumber("MM Turret Rotation", getRotations());
        SmartDashboard.putNumber("MM Turret Error Degrees", targetDegrees-getDegrees());
        SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
    }
}
