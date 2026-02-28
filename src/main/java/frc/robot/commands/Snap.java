package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class Snap extends Command {
    private final CommandSwerveDrivetrain swerve;

    private final SwerveRequest.FieldCentricFacingAngle alignRequest = new SwerveRequest.FieldCentricFacingAngle();

    private Rotation2d currentRotations;
    private Rotation2d targetRotations;
    private double radianVelocity;
    
    private final double angleDeadZone;

    public Snap(RobotContainer rob, double angleDeadband) {
        this.swerve = rob.drivetrain;
        this.angleDeadZone = angleDeadband;
    }
    
    @Override
    public void initialize() {
        currentRotations = swerve.getPoseMeters().getRotation();
    }
    
    public Rotation2d closestDegree(double rotations) {
        if(rotations > 315 || rotations < 45){
            return Rotation2d.fromDegrees(0);
        } else if(rotations > 45 || rotations < 135){
            return Rotation2d.fromDegrees(90);
        } else if(rotations > 135 || rotations < 225){
            return  Rotation2d.fromDegrees(180);
        } 
         return Rotation2d.fromDegrees(270);
    }

    @Override
    public void execute() {
        currentRotations = swerve.getPoseMeters().getRotation();
        targetRotations = closestDegree(currentRotations.getDegrees());
        // Get rotation rate
        if ((targetRotations.getDegrees() + angleDeadZone) > currentRotations.getDegrees()) {
            radianVelocity = 0.2;
        } else if (((targetRotations.getDegrees()) - angleDeadZone) < currentRotations.getDegrees()) {
            radianVelocity = -0.2;
        }

        swerve.setControl(
            alignRequest
                .withHeadingPID(5, 0, 0.1)
                .withTargetDirection(targetRotations)
                // .withMaxAbsRotationalRate(0.2)
              //  .withTargetRateFeedforward(Units.degreesToRadians(36))
        );
    }

    @Override
    public boolean isFinished() {
        return swerve.getPoseMeters().getRotation().getDegrees() - targetRotations.getDegrees() < angleDeadZone;
    }

    // @Override
    // public void end(boolean interrupted) {
    //     swerve.setControl(alignRequest.withVelocityX(0).withVelocityY(0));
    // }
}
