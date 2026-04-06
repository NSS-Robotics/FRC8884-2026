package frc.lib;

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
        // double swipe trench
        public static final Pose2d leftNeutralStartPos = new Pose2d(12, 0.64, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralPastTrench = new Pose2d(10.622, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralPreSwipe1 = new Pose2d(7.900, 1, Rotation2d.fromDegrees(-110));
        public static final Pose2d leftNeutralPreSwipe1Close = new Pose2d(8.626, 1, Rotation2d.fromDegrees(-110));
        public static final Pose2d leftNeutralSwipe1 = new Pose2d(8.200, 3.672, Rotation2d.fromDegrees(-110));
        public static final Pose2d leftNeutralSwipe1Close = new Pose2d(8.626, 3.65, Rotation2d.fromDegrees(-110));
        public static final Pose2d leftNeutralShootPos = new Pose2d(12.929, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralPreSwipe2 = new Pose2d(10.133, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralSwipe2 = new Pose2d(10.433, 3.657, Rotation2d.fromDegrees(-90));
        public static final Pose2d leftNeutralSwipe3 = new Pose2d(8.0, 3.657, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralSwipe4 = new Pose2d(8.0, 2.5, Rotation2d.fromDegrees(45));
        public static final Pose2d leftNeutralPostBump = new Pose2d(13.0, 2.5, Rotation2d.fromDegrees(0));
        public static final Pose2d leftNeutralStartPosClose = new Pose2d(12.94, 0.712, Rotation2d.fromDegrees(0)); // not auto start pos
        // 2056 auto: start -> pasttrench -> preswip1 ->swip1 -> pose before bump? -> post bump (pretty sure) -> back to start pos

        // middle: shoot + depot
        public static final Pose2d middlePreDepot = new Pose2d();
        public static final Pose2d middleInsideDepot = new Pose2d();

        // delayed bump: wait 5 seconds then go over bump
    }   

    private final RobotContainer rob;

    public boolean isRightFlipped = true; // blue left, red right is true
    public boolean isAllianceFlipped = false;
    public boolean isCloser = false; // closer swipe1 is true

    // AUTOES
    public SequentialCommandGroup redLeftNeutral;
    public SequentialCommandGroup middleDepot;

    public Autoes(RobotContainer rob) {
        this.rob = rob;
    }

    private void buildAuto() {
        redLeftNeutral = new SequentialCommandGroup(
            new InstantCommand(() -> rob.turret.autoShoot = true),
            new InstantCommand(rob.shooterHood::enableAutoDecap),
            new ParallelCommandGroup(
                alignTo(rob, Waypoints.leftNeutralPreSwipe1, 0.5, 6.0),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> rob.intakePivot.setPosition(15.9))
                    ),
                    new InstantCommand(() -> rob.intake.setVelocity(5000))
                )
            ),
            alignTo(rob, Waypoints.leftNeutralSwipe1, 0.1, 2.5),
            alignTo(rob, Waypoints.leftNeutralPastTrench, 0.2, 4),
            alignTo(rob, Waypoints.leftNeutralShootPos, 0.1, 4),
            new InstantCommand(rob.shooterHood::disableAutoDecap),
            new InstantCommand(() -> rob.turret.autoShoot = true),
            new ParallelRaceGroup(
                new WaitCommand(3.5),
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
                    alignTo(rob, Waypoints.leftNeutralPreSwipe2, 0.5, 4.0),
                    alignTo(rob, Waypoints.leftNeutralSwipe2, 0.1, 3)
                ),
                new InstantCommand(() -> rob.intakePivot.setPosition(15.9)),
                new InstantCommand(() -> rob.intake.setVelocity(5000))
            ),
            alignTo(rob, Waypoints.leftNeutralSwipe4, 0.2, 5),
            alignTo(rob, Waypoints.leftNeutralPostBump, 0.3, 5), 
          
            // alignTo(rob, Waypoints.leftNeutralPastTrench, 0.2, 4),
            // alignTo(rob, Waypoints.leftNeutralShootPos, 0.1, 4),
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

        middleDepot = new SequentialCommandGroup(
            // start facing hub
            // drive back/lower hopper
            // shoot
            // near depot/raise intake
            // depot/lower intake
            // intake
            // shoot
        );
    }

    public Command runAuto(RobotContainer rob, String autoToRun) {
        switch (autoToRun) {
        case "blueleft":
            isAllianceFlipped = true;
            isRightFlipped = true;
            break;
        case "blueright":
            isAllianceFlipped = true;
            isRightFlipped = false;
            break;
        case "redright":
            isAllianceFlipped = false;
            isRightFlipped = true;
            break;
        case "redleft":
            isAllianceFlipped = false;
            isRightFlipped = false;
            break;
        default:
            isAllianceFlipped = false;
            isRightFlipped = false;
            break;
        }

        buildAuto();
        if (autoToRun != "middle") {
            Pose2d startingPos = Waypoints.leftNeutralStartPos;
            if (isAllianceFlipped) startingPos = mirrorAlliance(startingPos);
            if (isRightFlipped) startingPos = mirrorToRight(startingPos);

            rob.drivetrain.resetPose(startingPos);
        } else {
            // FIXME: get middle starting position
            // rob.drivetrain.resetPose(null);
        }

        SequentialCommandGroup auto = new SequentialCommandGroup();
        auto.addCommands(redLeftNeutral);
        return auto;
    }

    private Command alignTo(RobotContainer rob, Pose2d pose, double deadzone, double maxvel) {
        if (isRightFlipped) pose = mirrorToRight(pose);
        if (isAllianceFlipped) pose = mirrorAlliance(pose);
        if (isCloser && pose == Waypoints.leftNeutralSwipe1) pose = Waypoints.leftNeutralSwipe1Close;
        if (isCloser && pose == Waypoints.leftNeutralPreSwipe1) pose = Waypoints.leftNeutralPreSwipe1Close;

        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new DriveToPoint(rob, pose, deadzone),
                rob.drivetrain.driveToPoint(pose, maxvel)
            ),
            new InstantCommand(rob.drivetrain::stopSwerve)
        );
        
        // return new ParallelRaceGroup(
        //     new DriveToPoint(rob, pose, deadzone),
        //     rob.drivetrain.driveToPoint(pose, maxvel)
        // );
    }
    
    private static Pose2d mirrorToRight(Pose2d pose) {
        return new Pose2d(pose.getX(), Constants.fieldYMeter - pose.getY(), Rotation2d.fromDegrees(-pose.getRotation().getDegrees()));
    }

    private static Pose2d mirrorAlliance(Pose2d pose){
        return new Pose2d(Constants.fieldXMeter - pose.getX(), pose.getY(), Rotation2d.fromDegrees(180 - pose.getRotation().getDegrees()));
    }
}
