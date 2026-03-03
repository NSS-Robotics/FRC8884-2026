package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Snap extends Command {
    private final CommandSwerveDrivetrain swerve;

    private final SwerveRequest.FieldCentricFacingAngle alignRequest = new SwerveRequest.FieldCentricFacingAngle();

    private Rotation2d currentRotations;
    private Rotation2d targetRotations;
    private double radianVelocity;
    
    private final double angleDeadZone;
    private final double direction;

    public Snap(RobotContainer rob, double angleDeadband, double direction) {
        this.swerve = rob.drivetrain;
        this.angleDeadZone = angleDeadband;
        this.direction = direction;
    }
    
    @Override
    public void initialize() {
        currentRotations = swerve.getPoseMeters().getRotation();
    }
    
    public Rotation2d closestDegree() {
         return Rotation2d.fromDegrees(direction);
    }

    @Override
    public void execute() {
        currentRotations = swerve.getPoseMeters().getRotation();
        targetRotations = closestDegree();
        // Get rotation rate
        if ((targetRotations.getDegrees() + angleDeadZone) > currentRotations.getDegrees()) {
            radianVelocity = 0.3;
        } else if (((targetRotations.getDegrees()) - angleDeadZone) < currentRotations.getDegrees()) {
            radianVelocity = -0.3;
        }

        swerve.setControl(
            alignRequest
                .withHeadingPID(4, 0, 0)
                .withTargetDirection(targetRotations)
                // .withMaxAbsRotationalRate(0.2)
                // .withTargetRateFeedforward(Units.degreesToRadians(36))
        );
    }

    // @Override
    // public boolean isFinished() {
    //     double degreeError = Math.abs(swerve.getPoseMeters().getRotation().getDegrees() - targetRotations.getDegrees());
    //     return (degreeError < angleDeadZone) || 
    //            (degreeError > (360 - angleDeadZone));
    // }
}
