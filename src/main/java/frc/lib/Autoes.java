package frc.lib;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoIntakeUnjam;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.RunSpindexer;

public class Autoes {
    public static class Waypoints {
        public static final Pose2d leftNeutralPastTrench = new Pose2d(10.622, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralPreSwipe1 = new Pose2d(8.200, 1, Rotation2d.fromDegrees(-110));
        public static final Pose2d leftNeutralSwipe1 = new Pose2d(8.200, 3.672, Rotation2d.fromDegrees(-110));
        public static final Pose2d leftNeutralShootPos = new Pose2d(12.929, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralSwipe2 = new Pose2d(10.433, 3.657, Rotation2d.fromDegrees(-90));
    }

    private final RobotContainer rob;

    // AUTOES
    public final SequentialCommandGroup redLeftNeutral;
    public final SequentialCommandGroup redRightNeutral;
    // public final SequentialCommandGroup blueLeftNeutral;
    // public final SequentialCommandGroup blueRightNeutral;

    public Autoes(RobotContainer rob) {
        this.rob = rob;

        redLeftNeutral = new SequentialCommandGroup(
            new InstantCommand(rob.shooterHood::enableAutoDecap),
            new ParallelCommandGroup(
                alignTo(rob, Waypoints.leftNeutralPreSwipe1, 0.5, 5.0),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> rob.intakePivot.setPosition(15.9))
                    ),
                    new InstantCommand(() -> rob.intake.setVelocity(5000))
                )
            ),
            alignTo(rob, Waypoints.leftNeutralSwipe1, 0.1, 2.5),
            alignTo(rob, Waypoints.leftNeutralPastTrench, 0.5, 2.5),
            alignTo(rob, Waypoints.leftNeutralShootPos, 0.1, 2.5),

            new InstantCommand(() -> rob.turret.autoShoot = true),
            new InstantCommand(rob.shooterHood::disableAutoDecap),
            new ParallelRaceGroup(
                new WaitCommand(4),
                new RunSpindexer(rob),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5)
                )
            ),
            new InstantCommand(rob.shooterHood::enableAutoDecap),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    alignTo(rob, Waypoints.leftNeutralPastTrench, 0.5, 4.0),
                    alignTo(rob, Waypoints.leftNeutralSwipe2, 0.1, 2.5)
                ),
                new InstantCommand(() -> rob.intakePivot.setPosition(15.9)),
                new InstantCommand(() -> rob.intake.setVelocity(5000))
            ),
            alignTo(rob, Waypoints.leftNeutralPastTrench, 0.5, 2.5),
            alignTo(rob, Waypoints.leftNeutralShootPos, 0.1, 2.5),
            new InstantCommand(rob.shooterHood::disableAutoDecap),
            new InstantCommand(() -> rob.turret.autoShoot = true),
            new ParallelRaceGroup(
                new WaitCommand(4),
                new RunSpindexer(rob),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5)
                )
            )
        );

        redRightNeutral = new SequentialCommandGroup(
            new InstantCommand(rob.shooterHood::enableAutoDecap),
            new ParallelCommandGroup(
                alignTo(rob, mirrorToRight(Waypoints.leftNeutralPreSwipe1), 0.5, 5.0),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> rob.intakePivot.setPosition(15.9))
                    ),
                    new InstantCommand(() -> rob.intake.setVelocity(5000))
                )
            ),
            alignTo(rob, mirrorToRight(Waypoints.leftNeutralSwipe1), 0.1, 2.5),
            alignTo(rob, mirrorToRight(Waypoints.leftNeutralPastTrench), 0.5, 2.5),
            alignTo(rob, mirrorToRight(Waypoints.leftNeutralShootPos), 0.1, 2.5),

            new InstantCommand(() -> rob.turret.autoShoot = true),
            new InstantCommand(rob.shooterHood::disableAutoDecap),
            new ParallelRaceGroup(
                new WaitCommand(4),
                new RunSpindexer(rob),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5)
                )
            ),
            new InstantCommand(rob.shooterHood::enableAutoDecap),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    alignTo(rob, mirrorToRight(Waypoints.leftNeutralPastTrench), 0.5, 4.0),
                    alignTo(rob, mirrorToRight(Waypoints.leftNeutralSwipe2), 0.1, 2.5)
                ),
                new InstantCommand(() -> rob.intakePivot.setPosition(15.9)),
                new InstantCommand(() -> rob.intake.setVelocity(5000))
            ),
            alignTo(rob, mirrorToRight(Waypoints.leftNeutralPastTrench), 0.5, 2.5),
            alignTo(rob, mirrorToRight(Waypoints.leftNeutralShootPos), 0.1, 2.5),
            new InstantCommand(rob.shooterHood::disableAutoDecap),
            new InstantCommand(() -> rob.turret.autoShoot = true),
            new ParallelRaceGroup(
                new WaitCommand(4),
                new RunSpindexer(rob),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(0.5)
                )
            )
        );

        // blueLeftNeutral = new SequentialCommandGroup(
        //     new InstantCommand(rob.shooterHood::enableAutoDecap),
        //     new ParallelCommandGroup(
        //         alignTo(rob, mirrorToRight(Waypoints.leftNeutralPreSwipe1), 0.5, 5.0),
        //         new SequentialCommandGroup(
        //             new ParallelDeadlineGroup(
        //                 new WaitCommand(1),
        //                 new InstantCommand(() -> rob.intakePivot.setPosition(15.9))
        //             ),
        //             new InstantCommand(() -> rob.intake.setVelocity(5000))
        //         )
        //     ),
        //     alignTo(rob, mirrorToRight(Waypoints.leftNeutralSwipe1), 0.1, 2.5),
        //     alignTo(rob, mirrorToRight(Waypoints.leftNeutralPastTrench), 0.5, 2.5),
        //     alignTo(rob, mirrorToRight(Waypoints.leftNeutralShootPos), 0.1, 2.5),

        //     new InstantCommand(() -> rob.turret.autoShoot = true),
        //     new InstantCommand(rob.shooterHood::disableAutoDecap),
        //     new ParallelRaceGroup(
        //         new WaitCommand(4),
        //         new RunSpindexer(rob),
        //         new SequentialCommandGroup(
        //             new WaitCommand(1),
        //             new ParallelRaceGroup(
        //                 new AutoIntakeUnjam(rob),
        //                 new WaitCommand(0.5)
        //             ),
        //             new WaitCommand(0.5),
        //             new ParallelRaceGroup(
        //                 new AutoIntakeUnjam(rob),
        //                 new WaitCommand(0.5)
        //             ),
        //             new WaitCommand(0.5),
        //             new ParallelRaceGroup(
        //                 new AutoIntakeUnjam(rob),
        //                 new WaitCommand(0.5)
        //             ),
        //             new WaitCommand(0.5)
        //         )
        //     ),
        //     new InstantCommand(rob.shooterHood::enableAutoDecap),
        //     new ParallelCommandGroup(
        //         new SequentialCommandGroup(
        //             alignTo(rob, mirrorToRight(Waypoints.leftNeutralPastTrench), 0.5, 4.0),
        //             alignTo(rob, mirrorToRight(Waypoints.leftNeutralSwipe2), 0.1, 2.5)
        //         ),
        //         new InstantCommand(() -> rob.intakePivot.setPosition(15.9)),
        //         new InstantCommand(() -> rob.intake.setVelocity(5000))
        //     ),
        //     alignTo(rob, mirrorToRight(Waypoints.leftNeutralPastTrench), 0.5, 2.5),
        //     alignTo(rob, mirrorToRight(Waypoints.leftNeutralShootPos), 0.1, 2.5),
        //     new InstantCommand(rob.shooterHood::disableAutoDecap),
        //     new InstantCommand(() -> rob.turret.autoShoot = true),
        //     new ParallelRaceGroup(
        //         new WaitCommand(4),
        //         new RunSpindexer(rob),
        //         new SequentialCommandGroup(
        //             new WaitCommand(1),
        //             new ParallelRaceGroup(
        //                 new AutoIntakeUnjam(rob),
        //                 new WaitCommand(0.5)
        //             ),
        //             new WaitCommand(0.5),
        //             new ParallelRaceGroup(
        //                 new AutoIntakeUnjam(rob),
        //                 new WaitCommand(0.5)
        //             ),
        //             new WaitCommand(0.5),
        //             new ParallelRaceGroup(
        //                 new AutoIntakeUnjam(rob),
        //                 new WaitCommand(0.5)
        //             ),
        //             new WaitCommand(0.5)
        //         )
        //     )
        // );
    }

    public Command runAuto(RobotContainer rob /*, Autos.AUTO_TYPE chosenAuto */) {
        SequentialCommandGroup auto = new SequentialCommandGroup();
        // auto.addCommands(redLeftNeutral);
        auto.addCommands(redRightNeutral);

        return auto;
    }

    private static Command alignTo(RobotContainer rob, Pose2d pose, double deadzone, double maxvel) {
        return new ParallelRaceGroup(
            new DriveToPoint(rob, pose, deadzone),
            rob.drivetrain.driveToPoint(pose, maxvel)
        );
    }
    
    private static Pose2d mirrorToRight(Pose2d pose) {
        return new Pose2d(pose.getX(), Constants.fieldYMeter - pose.getY(), Rotation2d.fromDegrees(-pose.getRotation().getDegrees()));
    }

    private static Pose2d mirrorAlliance(Pose2d pose){
        return new Pose2d(Constants.fieldXMeter - pose.getX(), pose.getY(), Rotation2d.fromDegrees(180 - pose.getRotation().getDegrees()));
    }
}
