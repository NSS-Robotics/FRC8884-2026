package frc.robot.subsystems;

import org.opencv.core.Mat;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final Flywheels flywheels;
    private final ShooterHood shooterHood;

    public double targetDegrees;
    private double distanceToTarget;
    public static double[] ans;

    private final Translation2d offsetFromRobotCenter;

    private double manualOffset = 0;

    public Pose2d turretLocation = new Pose2d();

    public MotionMagicTurret(RobotContainer rob, Translation2d translation) {
        this.intakePivot = rob.intakePivot;
        this.swerve = rob.drivetrain;
        this.rob = rob;
        this.offsetFromRobotCenter = translation;
        this.flywheels = rob.flywheels;
        this.shooterHood = rob.shooterHood;

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
        motionMagicConfigs.MotionMagicCruiseVelocity = 30; // maximum for an x60 is 100rps
        motionMagicConfigs.MotionMagicAcceleration = 30*3; // 0.5s to reach, *5 to take 0.2 sec, allegedy
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
        if (angle < Constants.Turret.rotationLimit1) {
            angle += 360.0;
        } else if (angle >= Constants.Turret.rotationLimit2) {
            angle -= 360.0;
        }
        return angle;
    }

    private Pose2d calculateTurretOffset(double xOffset, double yOffset, double robotRadians){
        Translation2d turretoffset = new Translation2d(xOffset, -yOffset).rotateBy(new Rotation2d(robotRadians));

        // x and y axis inverted
        /* 
        double d = Math.hypot(xOffset, yOffset);    
        double theta = Math.atan2(yOffset,xOffset);
        //double alpha = theta + robotRadians - Math.PI/2; 
        // double alpha = theta + robotRadians; 
        double alpha = robotRadians + theta + Math.PI;

        double x = d * Math.cos(alpha);
        double y = d * Math.sin(alpha);
        */
        //double x = xOffset  * Math.cos(robotRadians) - yOffset*Math.sin(robotRadians);
        //double y = xOffset  * Math.sin(robotRadians) + yOffset*Math.cos(robotRadians);
        double x = turretoffset.getX();
        double y = turretoffset.getY();

        //SmartDashboard.putNumber("MMT robotdeg", alpha*180/Math.PI);
        SmartDashboard.putNumber("MMT X", x);
        SmartDashboard.putNumber("MMT Y", y);
        return new Pose2d(x, y, null);
    }

    public double[] calculateMovingShot(double shootingVelocity, double theta, Pose2d robotPose, Pose2d targetPose){
        
        ChassisSpeeds speeds = swerve.getState().Speeds;
        Translation2d robotVelocityVector = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double robotSpeed = robotVelocityVector.getNorm();
        if (robotSpeed < 0.001){
            return new double[]{shootingVelocity, theta, 0};
        }
        Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
        double targetAngle = toTarget.getAngle().getRadians() + Math.PI;
        double robotAngle = robotVelocityVector.getAngle().getRadians() + Math.PI;
        double phi = robotAngle-targetAngle; 
        phi = Math.atan2(Math.sin(phi), Math.cos(phi));

        double thetar = Math.toRadians(theta);

        double x = shootingVelocity * Math.cos(thetar) - robotSpeed * Math.cos(phi);
        double y = -robotSpeed * Math.sin(phi);
        double z = shootingVelocity * Math.sin(thetar);

        double newShootingVelocity = Math.sqrt(x * x + y * y + z * z);
        // double newTheta = Math.toDegrees(Math.atan(z / x));
        // double newPhi = Math.toDegrees(Math.atan(y / x));
        double newTheta = Math.toDegrees(Math.atan2(z, x));
        double newPhi = Math.toDegrees(Math.atan2(y, x));

        SmartDashboard.putNumber("MMT X SPEED", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("MMT Y SPEED", speeds.vyMetersPerSecond);

        SmartDashboard.putNumber("MMT ROBOT SPEED", robotSpeed);
        SmartDashboard.putNumber("MMT THETA", theta);
        SmartDashboard.putNumber("MMT TARGET ANGLE", targetAngle * 180 / Math.PI);
        SmartDashboard.putNumber("MMT ROBOT ANGLE", robotAngle*180/Math.PI);
        SmartDashboard.putNumber("MMT PHI", phi*180/Math.PI);
        return new double[]{newShootingVelocity, newTheta, newPhi};
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

        double shooterVelocity = rpmToVelocity(flywheels.rpmToHub(distanceToTarget), Constants.Flywheel.flywheelRadiusMeters);
        SmartDashboard.putNumber("MMT SHOOTER VELOCITY", shooterVelocity);
        // double shooterVelocity = 15;
        // double shooterVelocity = flywheels.getVelocity();
        
        double theta = 90 - (Constants.Hood.irlHoodDegrees + shooterHood.getDegrees());
        Pose2d robotPose = swerve.getPoseMeters();
        Pose2d targetPose = Constants.targetPositionRed; // TODO: BE ABLE TO CHANGE RED AND BLUE SIDE

        ans = calculateMovingShot(shooterVelocity, theta, robotPose, targetPose); 
        SmartDashboard.putNumber("MMT TURRET COMPENSATION", ans[2]);


        double degreesToTarget = wrapAngleDegrees(offset - aimAngle 
                                                  - (currentPos.getRotation().getDegrees() - targetPos.getRotation().getDegrees()) - ans[2]); 
    
        SmartDashboard.putNumber("MM Turret degrees to target", degreesToTarget);
        SmartDashboard.putNumber("MM Turret Offset x", turretOffset.getX());
        SmartDashboard.putNumber("MM Turret Offset y", turretOffset.getY());
        
        setDegrees(degreesToTarget+manualOffset);
    }

    public double distanceToTime(double distance){
        return -0.18 * Math.pow(distance, 2) + 1.31 * distance - 1.05; // TODO: GET 6-8 METERS CALLI 
    }

    public double futureDistance(Pose2d currentPose, double distance, Pose2d targetPose){
        ChassisSpeeds speeds = swerve.getState().Speeds;
        Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double time = distanceToTime(distance);

        Pose2d newPose = new Pose2d(currentPose.getX() - robotVelocity.getX() * time,
                                    currentPose.getY() - robotVelocity.getY() * time,
                                    null);

        double xDistance = newPose.getX() - targetPose.getX();
        double yDistance = newPose.getY() - targetPose.getY();

        double newDistance = Math.hypot(xDistance, yDistance);
        SmartDashboard.putNumber("MMT DISTANCE COMPENSATION", newDistance);

        return newDistance;
    }


    public double rpmToVelocity(double rpm, double wheelRadiusMeters){ // IDK???
        double angularVelocity = rpm*2*Math.PI/60; // rad/s
        return angularVelocity*wheelRadiusMeters * 0.85;

    }

    public double velocityToRpm(double velocity, double wheelRadiusMeters){
        return velocity * 60 / (2 * Math.PI * wheelRadiusMeters);
    }

    public void setDistanceToTarget(double dist) {
        distanceToTarget = dist;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public void setManualOffset(double newManualOffset) {
        manualOffset = newManualOffset;
    }

    public double getManualOffset() {
        return manualOffset;
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

    // public boolean willGetDecapitated(){
    //     double x = swerve.getPoseMeters().getX();
    //     double y = swerve.getPoseMeters().getY();

    //     if (rob.isRed()){
    //         return (x < SMTH && x > SMTHELSE) &&  y < SMTHE || y > y;
    //     }
    // }

    @Override
    public void periodic() {
        // point turret at target
        // see: celtx 2022 ShooterSubsystem.java line 339

        // if (!isInAllianceZone()) { 
        //     pointAtTarget(rob.lobTarget);
        // } else {      
        //     if (SmartDashboard.getBoolean("Target Red Hub", true)) {
        //         pointAtTarget(Constants.targetPositionRed);
        //     } else {
        //         pointAtTarget(Constants.targetPositionBlue);
        //     }     
        // }
        // pointAtTarget(Constants.targetPositionBlue);
         pointAtTarget(Constants.targetPositionRed); // TODO: GET THIS 

        // if(!canMove()){
        //     stop();
        // }

        SmartDashboard.putNumber("MM Turret Angle Degrees", getDegrees());
        SmartDashboard.putNumber("MM Turret Rotation", getRotations());
        SmartDashboard.putNumber("MM Turret Error Degrees", targetDegrees-getDegrees());
        SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
        SmartDashboard.putNumber("MMT manual offset", manualOffset);
        SmartDashboard.putNumber("MMT PREDICTED DISTANCE", futureDistance(swerve.getPoseMeters(), distanceToTarget, Constants.targetPositionRed));
    }
}
