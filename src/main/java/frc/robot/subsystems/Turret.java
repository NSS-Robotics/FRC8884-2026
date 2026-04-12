package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor = new TalonFX(Constants.CanIDs.turretMotorID, new CANBus("Jeff"));
    private final CANcoder turretEncoder = new CANcoder(Constants.CanIDs.turretEncoderID, new CANBus("Jeff"));
    
    // private MotionMagicDutyCycle mmdc = new MotionMagicDutyCycle(0).withSlot(0);
    // private PositionVoltage pv = new PositionVoltage(0).withSlot(0);
    private final PositionTorqueCurrentFOC pt = new PositionTorqueCurrentFOC(0).withSlot(0);

    private Slot0Configs pidConfigs_0 = new Slot0Configs();
    private Slot1Configs pidConfigs_1 = new Slot1Configs();

    private final RobotContainer rob;
    private final CommandSwerveDrivetrain swerve;
    private final IntakePivot intakePivot;
    private final Flywheels flywheels;
    private final ShooterHood shooterHood;

    public double targetDegrees;
    private double distanceToTarget;
    public double[] ans;
    public double previousVelocityX = 0;
    public double previousVelocityY = 0;
    public double previousTime = 1;
    public double manualDistanceOffset = 0;

    private final Translation2d offsetFromRobotCenter;

    public double manualOffsetX = 0.0;
    public double manualOffsetY = 0.0;

    public Pose2d turretLocation = new Pose2d();

    public boolean autoShoot = false;
    public boolean isdocking = false;

    private final DoubleLogEntry distanceLog;
    private final DoubleLogEntry errorLog;

    public Turret(RobotContainer rob, Translation2d translation) {
        this.intakePivot = rob.intakePivot;
        this.swerve = rob.drivetrain;
        this.rob = rob;
        this.offsetFromRobotCenter = translation;
        this.flywheels = rob.flywheels;
        this.shooterHood = rob.shooterHood;

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = Constants.Turret.encoderOffset;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        turretEncoder.clearStickyFaults();
        turretEncoder.getConfigurator().apply(encoderConfig);

        TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
        turretMotorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        turretMotorConfig.Feedback.RotorToSensorRatio = Constants.Turret.motorToEncoderRatio; 
        turretMotorConfig.Feedback.SensorToMechanismRatio = Constants.Turret.gearRatioWithAbsoluteEncoder;
        turretMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(40).withPeakReverseTorqueCurrent(-40);

        CurrentLimitsConfigs turretMotorCurrentLimitsConfigs = turretMotorConfig.CurrentLimits;
        turretMotorCurrentLimitsConfigs.StatorCurrentLimit = 40;
        turretMotorCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
        pidConfigs_0 = turretMotorConfig.Slot0;
        pidConfigs_0.kP = Constants.Turret.kP_0;
        pidConfigs_0.kI = Constants.Turret.kI_0;
        pidConfigs_0.kD = Constants.Turret.kD_0;
        pidConfigs_0.kS = Constants.Turret.kS_0;
        pidConfigs_0.kV = Constants.Turret.kV_0;
        pidConfigs_0.kA = Constants.Turret.kA_0;

        pidConfigs_1 = turretMotorConfig.Slot1;
        pidConfigs_1.kP = Constants.Turret.kP_1;
        pidConfigs_1.kI = Constants.Turret.kI_1;
        pidConfigs_1.kD = Constants.Turret.kD_1;
        pidConfigs_1.kS = Constants.Turret.kS_1;
        pidConfigs_1.kV = Constants.Turret.kV_1;
        pidConfigs_1.kA = Constants.Turret.kA_1;  

        turretMotor.clearStickyFaults();
        turretMotor.getConfigurator().apply(turretMotorConfig);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);

        SmartDashboard.putNumber("turret p", 0);
        SmartDashboard.putNumber("turret i", 0);
        SmartDashboard.putNumber("turret d", 0);
        SmartDashboard.putNumber("turret s", 0);
        SmartDashboard.putNumber("turret v", 0);
        SmartDashboard.putNumber("turret a", 0);
        SmartDashboard.putNumber("turret setpoint", 0);

        distanceLog = new DoubleLogEntry(rob.log, "/turret/distance");
        errorLog = new DoubleLogEntry(rob.log, "/turret/errordegrees");
    }

    public void setRotations(double rotations, int slot) {
        
        turretMotor.setControl(pt.withSlot(slot).withPosition(rotations));
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
        //double rotations = degrees * Constants.Turret.gearRatioWithAbsoluteEncoder / 360; 
        double rotations = degrees / 360; 

        SmartDashboard.putNumber("MM Turret Target Degrees", degrees);
        double error = degrees - getDegrees();
        int slot = Math.abs(error) < 45.0  ? 0 : 1;  

        setRotations(rotations, slot);
    }

    public double getDegrees() {
        //return 360 * getRotations() / Constants.Turret.gearRatioWithAbsoluteEncoder;
        return 360 * getRotations();
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

    public double[] calculateMovingShot(double shootingVelocity, double theta, Pose2d robotPose, Pose2d targetPose){ // probably remove soon...
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

        newTheta = 90-Constants.Hood.irlHoodDegrees - newTheta;

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

    public Pose2d calculateOffsetTarget(Pose2d targetPos) {
        if (rob.isRed()) {
            return new Pose2d(targetPos.getX() + manualOffsetX, targetPos.getY() + manualOffsetY, targetPos.getRotation());
        }

        return new Pose2d(targetPos.getX() - manualOffsetX, targetPos.getY() - manualOffsetY, targetPos.getRotation());
    }

    public void pointAtTarget(Pose2d targetPos, boolean useSOTM) {
        Pose2d currentPos = swerve.getPoseMeters();
        
        double robotRadians = currentPos.getRotation().getRadians(); 
        Pose2d turretOffset = calculateTurretOffset(offsetFromRobotCenter.getX(), offsetFromRobotCenter.getY(), robotRadians);
        Pose2d offsettedTarget = calculateOffsetTarget(targetPos);

        turretLocation = new Pose2d(currentPos.getX() + turretOffset.getX(), currentPos.getY() + turretOffset.getY(), Rotation2d.fromDegrees(0));

        double xDistance = currentPos.getX() + turretOffset.getX() - offsettedTarget.getX();
        double yDistance = currentPos.getY() + turretOffset.getY() - offsettedTarget.getY();  

        setDistanceToTarget(Math.hypot(xDistance, yDistance));

        double sotmVI[] = sotmPredictVelocityIteration(currentPos, distanceToTarget, targetPos, useSOTM);

        double futureDegrees = sotmVI[1];

        double degreesToTarget = wrapAngleDegrees(futureDegrees
                                                  - (currentPos.getRotation().getDegrees() - offsettedTarget.getRotation().getDegrees()) 
                                                  ); 
        
        SmartDashboard.putNumber("MM Turret degrees to target", degreesToTarget);
        SmartDashboard.putNumber("MM Turret Offset x", turretOffset.getX());
        SmartDashboard.putNumber("MM Turret Offset y", turretOffset.getY());

        setDegrees(degreesToTarget);
    }

    public double distanceToTime(double distance){
        return Math.min(Math.max(0.03 * Math.pow(distance, 3) - 0.37 * Math.pow(distance, 2) + 1.33 * distance - 0.43, 1), 2) + 0.03;
        // return Math.min(Math.max(0.00404 * Math.pow(distance, 4) - 0.0483 * Math.pow(distance, 3) + 0.179 * Math.pow(distance, 2) - 0.107 * distance + 0.961, 1), 2) + 0.03;
    }   

    public double getAcceleration(double v1, double v2, double t1, double t2) {
        if (t1 == t2){
            return 0;
        }
        return (v1-v2)/(t1-t2);
    }

    public double[] sotmPredictVelocityIteration(Pose2d currentPose, double distance, Pose2d targetPose, boolean useSOTM) {
        targetPose = calculateOffsetTarget(targetPose);
        ChassisSpeeds speeds = swerve.getState().Speeds;
        double robotRadians = currentPose.getRotation().getRadians(); 
        Pose2d turretOffset = calculateTurretOffset(offsetFromRobotCenter.getX(), offsetFromRobotCenter.getY(), robotRadians);
        Pose2d turretPose = new Pose2d(currentPose.getX() + turretOffset.getX(), currentPose.getY() + turretOffset.getY(), null);

        double tangentalVX = -speeds.omegaRadiansPerSecond * turretOffset.getY();
        double tangentalVY = speeds.omegaRadiansPerSecond * turretOffset.getX();

        double vx = speeds.vxMetersPerSecond + tangentalVX;
        double vy = speeds.vyMetersPerSecond + tangentalVY;
  
        double turretVelocityX = vx * Math.cos(robotRadians) - vy * Math.sin(robotRadians); 
        double turretVelocityY = vx * Math.sin(robotRadians) + vy * Math.cos(robotRadians);   

        double lookaheadDistance = distance;
        Pose2d lookaheadTurretPose = turretPose;

        double prevDistance = 1000;

        for (int i = 0; i < 20; i++){ 
            double timeOfFlight = distanceToTime(lookaheadDistance);

            double dvX = turretVelocityX * timeOfFlight;
            double dvY = turretVelocityY * timeOfFlight;

            // lookaheadTurretPose = new Pose2d(turretPose.getX() + dvX, turretPose.getY() + dvY, null);
            lookaheadTurretPose = new Pose2d(turretPose.getTranslation().plus(new Translation2d(dvX, dvY)), null);
            
            // double xDistance = lookaheadTurretPose.getX() - targetPose.getX();
            // double yDistance = lookaheadTurretPose.getY() - targetPose.getY();

            // lookaheadDistance = Math.hypot(xDistance, yDistance);
            lookaheadDistance = targetPose.getTranslation().getDistance(lookaheadTurretPose.getTranslation());
            if (Math.abs(prevDistance - lookaheadDistance) < 0.001){
                break;
            }
            prevDistance = lookaheadDistance;
        }

        double xDistance = lookaheadTurretPose.getX() - targetPose.getX();
        double yDistance = lookaheadTurretPose.getY() - targetPose.getY();

        double newAimAngle = degreesToSet(xDistance, yDistance);

        double newDistance = Math.hypot(xDistance, yDistance);
        SmartDashboard.putNumber("MMT DISTANCE COMPENSATION", newDistance);
        SmartDashboard.putNumber("MMT NEW ANGLE", newAimAngle);
        SmartDashboard.putNumber("TIME CALLI", distanceToTime(distance));

        newDistance += manualDistanceOffset;

        if(useSOTM){
            return new double[]{newDistance, newAimAngle};
        }else{
            return new double[]{distance, degreesToSet(turretPose.getX()- targetPose.getX(), turretPose.getY()- targetPose.getY())};
        }
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

    public void setManualOffsetX(double newManualOffset) {
        manualOffsetX = newManualOffset;
    }

    public double getManualOffsetX() {
        return manualOffsetX;
    }
    public void setManualOffsetY(double newManualOffset) {
        manualOffsetY = newManualOffset;
    }

    public double getManualOffsetY() {
        return manualOffsetY;
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

        if (!intakePivot.isIntakeUp()) {
            if (!isInAllianceZone()) { 
                if (rob.isRed()){
                    if (swerve.getPoseMeters().getY() > 4){
                        pointAtTarget(Constants.leftCornerRed, false);
                    } else{
                        pointAtTarget(Constants.rightCornerRed, false);
                    }
                } else {
                    if (swerve.getPoseMeters().getY() > 4){
                        pointAtTarget(Constants.leftCornerBlue, false);
                    } else{
                        pointAtTarget(Constants.rightCornerBlue, false);
                    }
                }
            } else {      
                if (rob.isRed()){
                    // pointAtTarget(Constants.targetPositionBlue);

                    pointAtTarget(Constants.targetPositionRed, true);
                } else {
                    pointAtTarget(Constants.targetPositionBlue, true);
                    // pointAtTarget(Constants.targetPositionRed);

                }
            }
        }

        // if (!intakePivot.isIntakeUp()){
        //     pointAtTarget(Constants.targetPositionRed, true);
        // }
        
        //else {
        //     setDegrees(90);
        //     isdocking = false;
        // }

        double sotmVI[] = sotmPredictVelocityIteration(swerve.getPoseMeters(), distanceToTarget, rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue, true); 

        if (autoShoot) {
            flywheels.setVelocity(flywheels.rpmToHub(sotmVI[0])); // FIXME: comment back in after tuning
            shooterHood.setDegrees();
        }

        SmartDashboard.putNumber("MMT SOTMVI DISTANCE", sotmVI[0]);
        SmartDashboard.putNumber("MMT SOTMVI ANGLE", sotmVI[1]);

        SmartDashboard.putNumber("MM Turret Angle Degrees", getDegrees());
        SmartDashboard.putNumber("MM Turret Rotation", getRotations());
        SmartDashboard.putNumber("MM Turret Error Degrees", targetDegrees-getDegrees());
        SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
        SmartDashboard.putNumber("MMT manual offset X", manualOffsetX);
        SmartDashboard.putNumber("MMT manual offset Y", manualOffsetY);

        SmartDashboard.putBoolean("ALLIANCEZONE", isInAllianceZone());

        SmartDashboard.putNumber("Manual Distance Offset", manualDistanceOffset);

        // Log to logs
        errorLog.append(targetDegrees-getDegrees());
        distanceLog.append(distanceToTarget);

        // for pid tuning
        // double p = SmartDashboard.getNumber("turret p", 0);
        // double i = SmartDashboard.getNumber("turret i", 0);
        // double d = SmartDashboard.getNumber("turret d", 0);
        // double s = SmartDashboard.getNumber("turret s", 0);
        // double v = SmartDashboard.getNumber("turret v", 0);
        // double a = SmartDashboard.getNumber("turret a", 0);

        // boolean isChanged = (
        //     p != pidConfigs_0.kP ||
        //     i != pidConfigs_0.kI ||
        //     d != pidConfigs_0.kD ||
        //     s != pidConfigs_0.kS ||
        //     v != pidConfigs_0.kV ||
        //     a != pidConfigs_0.kA);
        
        // if (isChanged) {
        //     pidConfigs_0.kP = p;
        //     pidConfigs_0.kI = i;
        //     pidConfigs_0.kD = d;
        //     pidConfigs_0.kS = s;
        //     pidConfigs_0.kV = v;
        //     pidConfigs_0.kA = a;
        //     turretMotor.getConfigurator().apply(pidConfigs_0);
        // }

        // setDegrees(SmartDashboard.getNumber("turret setpoint", 0));
    }
}