package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPoint extends Command {
    private final Pose2d targetPose;
    private final CommandSwerveDrivetrain swerve;
    private final double deadzone;

    public DriveToPoint(RobotContainer rob, Pose2d targetPose, double deadzone) {
        swerve = rob.drivetrain;
        this.targetPose = targetPose;
        this.deadzone = deadzone;
    }

    @Override
    public boolean isFinished() {
        double distanceError = targetPose.getTranslation().getDistance(swerve.getPoseMeters().getTranslation());

        return distanceError < deadzone;
    }
}