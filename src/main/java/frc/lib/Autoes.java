package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToPoint;

public class Autoes {
    // makeAlignCmd(pose)
    // command()

    public static class Waypoints {
        // public static final Pose2d leftNeutralStart;
        public static final Pose2d leftNeutralPastTrench = new Pose2d(10.622, 0.712, Rotation2d.fromDegrees(-90));
        public static final Pose2d leftNeutralPastTrenchFacingDriver = new Pose2d(10.622, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralPreTrench = new Pose2d(12.929, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralSwipe1 = new Pose2d(8.200, 3.672, Rotation2d.fromDegrees(-110));
        public static final Pose2d leftNeutralShootPos = new Pose2d(14.245, 0.712, Rotation2d.fromDegrees(-135));
        public static final Pose2d leftNeutralSwipe2 = new Pose2d(10.433, 3.657, Rotation2d.fromDegrees(-90));
    }

    // AUTOES
    public static final Pose2d[] leftNeutral = {
        Waypoints.leftNeutralPastTrench,
        Waypoints.leftNeutralSwipe1,
        Waypoints.leftNeutralPastTrenchFacingDriver,
        Waypoints.leftNeutralPreTrench,
        Waypoints.leftNeutralShootPos,
        Waypoints.leftNeutralPastTrenchFacingDriver,
        Waypoints.leftNeutralSwipe2,
        Waypoints.leftNeutralPastTrenchFacingDriver,
        Waypoints.leftNeutralPreTrench,
        Waypoints.leftNeutralShootPos,
    };

    // public static final Command[] leftNeutral = {
        // alignTo(rob, Waypoints.leftNeutralPastTrench),
        // alignTo(rob, Waypoints.leftNeutralSwipe1),
        // alignTo(rob, Waypoints.leftNeutralPastTrench),
        // alignTo(rob, Waypoints.leftNeutralShootPos),
        // alignTo(rob, Waypoints.leftNeutralPastTrench),
        // alignTo(rob, Waypoints.leftNeutralSwipe2),
        // alignTo(rob, Waypoints.leftNeutralPastTrench),
        // alignTo(rob, Waypoints.leftNeutralShootPos),
    // }


    public static Command runAuto(RobotContainer rob /*, Autos.AUTO_TYPE chosenAuto */) {
        SequentialCommandGroup auto = new SequentialCommandGroup();
        auto.addCommands(new InstantCommand(rob.shooterHood::enableAutoDecap));

        for (var pose : leftNeutral) {
            auto.addCommands(new ParallelRaceGroup(
                new DriveToPoint(rob, pose, 0.1),
                rob.drivetrain.driveToPoint(pose)
            ));
        }

        return auto;
    }
}
