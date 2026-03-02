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
    public /* static */ double[] ans;
    public double previousVelocityX = 0;
    public double previousVelocityY = 0;
    public double previousTime = 1;

    private final Translation2d offsetFromRobotCenter;

    private double manualOffset = 0;

    public Pose2d turretLocation = new Pose2d();

    public boolean autoShoot = false;

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
        motionMagicConfigs.MotionMagicJerk = 9999.0; // TODO: figure out if this is a good value

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
        double x = turretoffset.getX();
        double y = turretoffset.getY();

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

    public double degreesToSet(double xDistance, double yDistance){
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

        return offset-aimAngle;

    }

    public void pointAtTarget(Pose2d targetPos) { 
        Pose2d currentPos = swerve.getPoseMeters();
        
        double robotRadians = currentPos.getRotation().getRadians(); 
        Pose2d turretOffset = calculateTurretOffset(offsetFromRobotCenter.getX(), offsetFromRobotCenter.getY(), robotRadians);

        turretLocation = new Pose2d(currentPos.getX() + turretOffset.getX(), currentPos.getY() + turretOffset.getY(), Rotation2d.fromDegrees(0));

        double xDistance = currentPos.getX() + turretOffset.getX() - targetPos.getX();
        double yDistance = currentPos.getY() + turretOffset.getY() - targetPos.getY();  

        setDistanceToTarget(Math.hypot(xDistance, yDistance));

        /*double aimDegrees = degreesToSet(xDistance, yDistance);
        
        double shooterVelocity = rpmToVelocity(flywheels.rpmToHub(distanceToTarget), Constants.Flywheel.flywheelRadiusMeters);
        SmartDashboard.putNumber("MMT SHOOTER VELOCITY", shooterVelocity);
        // double shooterVelocity = 15;
        // double shooterVelocity = flywheels.getVelocity();
        
        double theta = 90 - (Constants.Hood.irlHoodDegrees + shooterHood.getDegrees());
        Pose2d robotPose = swerve.getPoseMeters();
        //Pose2d targetPose = Constants.targetPositionBlue; // TODO: BE ABLE TO CHANGE RED AND BLUE SIDE

        ans = calculateMovingShot(shooterVelocity, theta, robotPose, targetPos); 
        SmartDashboard.putNumber("MMT TURRET COMPENSATION", ans[2]);*/

        double future[] = aLookIntoTheFuture(currentPos, yDistance, targetPos);
        double futureDegrees = future[1];


        // double degreesToTarget = wrapAngleDegrees(aimDegrees
        //                                           - (currentPos.getRotation().getDegrees() - targetPos.getRotation().getDegrees()) 
        //                                           + ans[2]); //TODO + FOR BLUE - FOR RED *idk if this is still an issue, probably not

        double degreesToTarget = wrapAngleDegrees(futureDegrees
                                                  - (currentPos.getRotation().getDegrees() - targetPos.getRotation().getDegrees()) 
                                                  ); 
        
        SmartDashboard.putNumber("MM Turret degrees to target", degreesToTarget);
        SmartDashboard.putNumber("MM Turret Offset x", turretOffset.getX());
        SmartDashboard.putNumber("MM Turret Offset y", turretOffset.getY());

        setDegrees(degreesToTarget+manualOffset);
    }

    public double distanceToTime(double distance){
        return Math.min(Math.max(0.03 * Math.pow(distance, 2) + 0.01 * distance + 0.91, 1), 1.5);
    }   

    public double getAcceleration(double v1, double v2, double t1, double t2){
        if (t1 == t2){
            return 0;
        }
        return (v1-v2)/(t1-t2);
    }

    public double[] aLookIntoTheFuture(Pose2d currentPose, double distance, Pose2d targetPose){
        ChassisSpeeds speeds = swerve.getState().Speeds;
        Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double robotRadians = currentPose.getRotation().getRadians(); 
        Pose2d turretOffset = calculateTurretOffset(offsetFromRobotCenter.getX(), offsetFromRobotCenter.getY(), robotRadians);
        Pose2d turretPose = new Pose2d(currentPose.getX() + turretOffset.getX(), currentPose.getY() + turretOffset.getY(), null);
        double time = distanceToTime(distance);

        double turretVelocityX = speeds.vxMetersPerSecond;
        double turretVelocityY = speeds.vyMetersPerSecond;

        double lookaheadDistance = distance;
        Pose2d lookaheadTurretPose = turretPose;

        double accelerationX;
        double accelerationY;

        accelerationX = getAcceleration(robotVelocity.getX(), previousVelocityX, time, previousTime);
        previousVelocityX = robotVelocity.getX();
        accelerationY = getAcceleration(robotVelocity.getY(), previousVelocityY, time, previousTime);
        previousVelocityY = robotVelocity.getY();
        previousTime = time;

        double dvX = robotVelocity.getX() * time;
        double dvY = robotVelocity.getY() * time;
        double daX = accelerationX * time * time / 2;
        double daY = accelerationY * time * time / 2; 

        if (rob.isRed()){
            // turretVelocityX *= -1.0;
            // turretVelocityY *= -1.0;
            dvX *= -1.0;
            dvY *= -1.0;
            daX *= -1.0;
            daY *= -1.0;
        }

        // for (int i = 0; i < 10; i++){
        //     double timeOfFlight = distanceToTime(lookaheadDistance);
        //     double offsetX = turretVelocityX * timeOfFlight;
        //     double offsetY = turretVelocityY * timeOfFlight;

        //     double accelerationX;
        //     double accelerationY;
        //     accelerationX = getAcceleration(turretVelocityX, previousVelocityX, timeOfFlight, previousTime);
        //     previousVelocityX = turretVelocityX;
        //     accelerationY = getAcceleration(turretVelocityY, previousVelocityY, timeOfFlight, previousTime);
        //     previousVelocityY = turretVelocityY;
        //     previousTime = timeOfFlight;

        //     if (rob.isRed()){
        //         accelerationX *= -1.0;
        //         accelerationY *= -1.0;
        //     }
            
        //     lookaheadTurretPose = new Pose2d(turretPose.getX() + offsetX + accelerationX, turretPose.getY() + offsetY + accelerationY, null);
            
        //     double xDistance = lookaheadTurretPose.getX() - targetPose.getX();
        //     double yDistance = lookaheadTurretPose.getY() - targetPose.getY();

        //     lookaheadDistance = Math.hypot(xDistance, yDistance);
        // }

        Pose2d newPose = new Pose2d(currentPose.getX() + dvX + daX, 
                                    currentPose.getY() + dvY + daY,
                                    null);

        double xDistance = newPose.getX() + turretOffset.getX() - targetPose.getX();
        double yDistance = newPose.getY() + turretOffset.getY() - targetPose.getY(); 
        // double xDistance = lookaheadTurretPose.getX() - targetPose.getX();
        // double yDistance = lookaheadTurretPose.getY() - targetPose.getY();

        
        double newAimAngle = degreesToSet(xDistance, yDistance);

        double newDistance = Math.hypot(xDistance, yDistance);
        SmartDashboard.putNumber("MMT DISTANCE COMPENSATION", newDistance);
        SmartDashboard.putNumber("MMT NEW ANGLE", newAimAngle);
        SmartDashboard.putNumber("TIME CALLI", distanceToTime(distance));

        return new double[]{newDistance, newAimAngle};
    }

    public double rpmToVelocity(double rpm, double wheelRadiusMeters){
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
            return swerve.getPoseMeters().getX() > 12.5;
        }

        return swerve.getPoseMeters().getX() < 4;
    }

    @Override
    public void periodic() {
        // point turret at target
        // see: celtx 2022 ShooterSubsystem.java line 339

        if (!isInAllianceZone()) { 
            if (rob.isRed()){
                if (swerve.getPoseMeters().getY() > 4){
                    pointAtTarget(Constants.leftCornerRed);
                } else{
                    pointAtTarget(Constants.rightCornerRed);
                }
            } else{
                if (swerve.getPoseMeters().getY() > 4){
                    pointAtTarget(Constants.leftCornerBlue);
                } else{
                    pointAtTarget(Constants.rightCornerBlue);
                }
            }
        } else {      
            if (rob.isRed()){
                pointAtTarget(Constants.targetPositionRed);
            } else {
                pointAtTarget(Constants.targetPositionBlue);
            }     
        }

        double future[] = aLookIntoTheFuture(swerve.getPoseMeters(), distanceToTarget, rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue); 
 
        double futureDistance = future[0];
        double futureAngle = future[1];

        if (autoShoot) {
            flywheels.setVelocity(flywheels.rpmToHub(future[0]));
            shooterHood.setDegrees();
        }

        SmartDashboard.putNumber("MM Turret Angle Degrees", getDegrees());
        SmartDashboard.putNumber("MM Turret Rotation", getRotations());
        SmartDashboard.putNumber("MM Turret Error Degrees", targetDegrees-getDegrees());
        SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
        SmartDashboard.putNumber("MMT manual offset", manualOffset);
        SmartDashboard.putNumber("MMT PREDICTED DISTANCE", futureDistance);
        SmartDashboard.putNumber("MMT PREDICTED ANGLE", futureAngle); 
        SmartDashboard.putBoolean("ALLIANCEZONE", isInAllianceZone());
    }
}
