package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Align;

public class Autoes {
    // makeAlignCmd(pose)
    // command()

    // AUTOES
    public static final Pose2d[] leftNeutral = {
        new Pose2d(8.200, 3.672, Rotation2d.fromDegrees(-110)),

        new Pose2d(10.600, 0.645, Rotation2d.fromDegrees(-110)),
        new Pose2d(14.245, 0.812, Rotation2d.fromDegrees(-135)),

        new Pose2d(10.731, 0.695, Rotation2d.fromDegrees(-135)),
        new Pose2d(10.433, 3.657, Rotation2d.fromDegrees(-90)),

        new Pose2d(11.914, 0.536, Rotation2d.fromDegrees(-90)),
        new Pose2d(14.245, 0.812, Rotation2d.fromDegrees(-135)),
    };

    public static Command runAuto(RobotContainer rob /*, Autos.AUTO_TYPE chosenAuto */) {
        SequentialCommandGroup auto = new SequentialCommandGroup();
        for (var pose : leftNeutral) {
            auto.addCommands(new Align(rob, pose, 0.1));
        }

        return auto;
    }
}
