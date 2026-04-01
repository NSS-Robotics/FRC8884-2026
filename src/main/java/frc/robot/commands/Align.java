package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Align extends Command {
    private final CommandSwerveDrivetrain swerve;

    private final SwerveRequest.FieldCentricFacingAngle alignRequest = new SwerveRequest.FieldCentricFacingAngle();

    private Pose2d currentPos;
    private final Pose2d targetPos;
    private final double deadZone;

    public Align(RobotContainer rob, Pose2d targetPos, double deadband) {
        this.swerve = rob.drivetrain;
        this.targetPos = targetPos;
        this.deadZone = deadband;
    }

    public double mapDistanceToVelocity(double distance) {
        final double maxVel = 2.5;
        final double minVel = 0.08;

        distance = Math.abs(distance);
        if (distance > 1) distance = 1;

        return minVel + distance * (maxVel - minVel);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Target X", targetPos.getX());
        SmartDashboard.putNumber("Target Y", targetPos.getY());
        SmartDashboard.putNumber("Target R", targetPos.getRotation().getDegrees());

        double xVelocity = 0;
        double yVelocity = 0;
        // TODO: do we need this????
        // double radianVelocity = headingController.calculate(currentPos.getRotation().getRadians(), targetPos.getRotation().getRadians(), Timer.getFPGATimestamp());

        currentPos = swerve.getPoseMeters();

        // Get X velocity
        if ((targetPos.getX() + deadZone) > currentPos.getX()) {
            xVelocity = -mapDistanceToVelocity(currentPos.getX() - targetPos.getX());
        } else if ((targetPos.getX() - deadZone) < currentPos.getX()) {
            xVelocity = mapDistanceToVelocity(currentPos.getX() - targetPos.getX());
        }

        // Get Y velocity
        if ((targetPos.getY() + deadZone) > currentPos.getY()) {
            yVelocity = -mapDistanceToVelocity(currentPos.getY() - targetPos.getY());
        } else if ((targetPos.getY() - deadZone) < currentPos.getY()) {
            yVelocity = mapDistanceToVelocity(currentPos.getY() - targetPos.getY());
        }

        // TODO: do we need thsi??? (pt 2)
        // // Get rotation rate
        // if ((targetPos.getRotation().getDegrees() + angleDeadZone) > currentPos.getRotation().getDegrees()) {
        //     radianVelocity = 0.2;
        // } else if ((targetPos.getRotation().getDegrees() - angleDeadZone) < currentPos.getRotation().getDegrees()) {
        //     radianVelocity = -0.2;
        // }

        swerve.setControl(
            alignRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withDeadband(deadZone)
                .withHeadingPID(5, 0, 0.1)
                .withTargetDirection(targetPos.getRotation())
        );
    }

    @Override
    public boolean isFinished() {
        double distanceToTarget = Math.hypot(
            swerve.getPoseMeters().getX() - targetPos.getX(),
            swerve.getPoseMeters().getY() - targetPos.getY()
        );

        return distanceToTarget < deadZone;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(alignRequest.withVelocityX(0).withVelocityY(0));
    }
}