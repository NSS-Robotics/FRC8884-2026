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

        // ALWAYS IN THE POSITION OF LEFT RED, methods to flip sides and alliances
        public static final Pose2d startPos = new Pose2d(12.4, 0.64, Rotation2d.fromDegrees(0));
        public static final Pose2d pastTrench = new Pose2d(10.622, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d preSwipe1 = new Pose2d(8.2, 1, Rotation2d.fromDegrees(-90)); // go farther?
        //public static final Pose2d preSwipe1 = new Pose2d(7.900, 1, Rotation2d.fromDegrees(-90)); // go farther?

        public static final Pose2d preSwipe1Close = new Pose2d(8.626, 1, Rotation2d.fromDegrees(-110));
        public static final Pose2d preSwipe1Greedy = new Pose2d(7, 1, Rotation2d.fromDegrees(-90));
        public static final Pose2d swipe1 = new Pose2d(8.500, 3.672, Rotation2d.fromDegrees(-120));
        public static final Pose2d swipe1Close = new Pose2d(8.626, 3.65, Rotation2d.fromDegrees(-110));
        public static final Pose2d shootPos = new Pose2d(12.929, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d preSwipe2 = new Pose2d(9.633, 0.712, Rotation2d.fromDegrees(0));
        public static final Pose2d swipe2 = new Pose2d(9.933, 3.657, Rotation2d.fromDegrees(-90));
        public static final Pose2d swipe4 = new Pose2d(8.0, 2.5, Rotation2d.fromDegrees(45));
        public static final Pose2d postBump = new Pose2d(13.5, 2.5, Rotation2d.fromDegrees(0));
        public static final Pose2d startPosClose = new Pose2d(12.94, 0.712, Rotation2d.fromDegrees(0)); // not auto start pos
    
        // double swipe bump
        public static final Pose2d startPosRotated = new Pose2d(12.167, 0.463, Rotation2d.fromDegrees(-90)); // not in auto
        public static final Pose2d postBumpIntake = new Pose2d(10.5, 2.5, Rotation2d.fromDegrees(180));
        public static final Pose2d posterBumpIntake = new Pose2d(10.033, 2.5, Rotation2d.fromDegrees(180));
        public static final Pose2d postBumpIntake_s2 = new Pose2d(10.5, 2.5, Rotation2d.fromDegrees(0));

        public static final Pose2d preBump = new Pose2d(13.4, 2.6, Rotation2d.fromDegrees(180));
        public static final Pose2d bumpToTrenchTransition1 = new Pose2d(13.2, 2.262, Rotation2d.fromDegrees(120));
        public static final Pose2d bumpToTrenchTransition2 = new Pose2d(13.303, 1.6, Rotation2d.fromDegrees(60));
        public static final Pose2d bumpToTrenchTransition3 = new Pose2d(13.2, 0.75, Rotation2d.fromDegrees(0));

        // middle: shoot + depot
        public static final Pose2d middleStart = new Pose2d(13, 4, Rotation2d.fromDegrees(0));
        public static final Pose2d middlePreDepot = new Pose2d(14.5, 2.8, Rotation2d.fromDegrees(90));
        public static final Pose2d middleInsideDepot = new Pose2d(15.7, 2.118, Rotation2d.fromDegrees(180));

        // delayed bump: wait 5 seconds then go over bump (low priority)
    }   

    private RobotContainer rob;

    public boolean isRightFlipped = false; // blue left, red right is true
    public boolean isAllianceFlipped = false;
    public boolean isCloser = false; // closer swipe1 is true

    // AUTOES
    public SequentialCommandGroup redLeftDoubleSwipe;
    public SequentialCommandGroup middleDepot;
    public SequentialCommandGroup redLeftDoubleSwipeBump;
    public SequentialCommandGroup redLeftDoubleSwipe1Bump;

    public Autoes(RobotContainer rob) {
        this.rob = rob;
    }

    public void buildAuto() {

        redLeftDoubleSwipe1Bump = new SequentialCommandGroup(
            new InstantCommand(() -> rob.turret.autoShoot = true),
            new InstantCommand(rob.shooterHood::enableAutoDecap),
            new ParallelCommandGroup(
                alignTo(rob, Waypoints.preSwipe1, 0.5, 6.0),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> rob.intakePivot.setPosition(15.9))
                    ),
                    new InstantCommand(() -> rob.intake.setVelocity(5000))
                )
            ),
            alignTo(rob, Waypoints.swipe1, 0.1, 2.5),
            alignTo(rob, Waypoints.pastTrench, 0.2, 4),
            alignTo(rob, Waypoints.shootPos, 0.1, 4),
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
                    alignTo(rob, Waypoints.preSwipe2, 0.5, 4.0),
                    alignTo(rob, Waypoints.swipe2, 0.1, 3)
                ),
                new InstantCommand(() -> rob.intakePivot.setPosition(15.9)),
                new InstantCommand(() -> rob.intake.setVelocity(5000))
            ),
            alignTo(rob, Waypoints.swipe4, 0.2, 5),
            alignTo(rob, Waypoints.postBump, 0.3, 5), 
          
            // alignTo(rob, Waypoints.PastTrench, 0.2, 4),
            // alignTo(rob, Waypoints.ShootPos, 0.1, 4),
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

        redLeftDoubleSwipe = new SequentialCommandGroup(
            new InstantCommand(() -> rob.turret.autoShoot = true),
            new InstantCommand(rob.shooterHood::enableAutoDecap),
            new ParallelCommandGroup(
                alignTo(rob, Waypoints.preSwipe1, 0.5, 6.0),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> rob.intakePivot.setPosition(15.9))
                    ),
                    new InstantCommand(() -> rob.intake.setVelocity(5000))
                )
            ),
            alignTo(rob, Waypoints.swipe1, 0.1, 2.5),
            alignTo(rob, Waypoints.pastTrench, 0.2, 4),
            alignTo(rob, Waypoints.shootPos, 0.1, 4),
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
                    alignTo(rob, Waypoints.preSwipe2, 0.5, 4.0),
                    alignTo(rob, Waypoints.swipe2, 0.1, 3)
                ),
                new InstantCommand(() -> rob.intakePivot.setPosition(15.9)),
                new InstantCommand(() -> rob.intake.setVelocity(5000))
            ),
            alignTo(rob, Waypoints.pastTrench, 0.2, 4),
            alignTo(rob, Waypoints.shootPos, 0.1, 4),
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
            new InstantCommand(() -> rob.turret.autoShoot = true),
            // alignTo(rob, Waypoints.middleStart, 0.1, 5.0),
            new ParallelCommandGroup(
                alignTo(rob, Waypoints.middlePreDepot, 0.5, 2.0),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> rob.intakePivot.setPosition(15.9))
                    ),
                    new InstantCommand(() -> rob.intake.setVelocity(5000))
                )
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(2),
                new RunSpindexer(rob)
            ),
            alignTo(rob, Waypoints.middleInsideDepot, 0.1, 2.0),
            new RunSpindexer(rob)
        );

        redLeftDoubleSwipeBump = new SequentialCommandGroup(
            new InstantCommand(rob.shooterHood::enableAutoDecap),
            // alignTo(rob, Waypoints.startPosRotated, 0.1, 2.5),
            new InstantCommand(() -> rob.turret.autoShoot = true),

            new ParallelCommandGroup(
                // alignTo(rob, Waypoints.preSwipe1, 0.7, 6), // FIXME: this one prob better
                alignTo(rob, Waypoints.preSwipe1, 0.8, 6), 
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> rob.intakePivot.setPosition(15.9))
                    ),
                    new InstantCommand(() -> rob.intake.setVelocity(5000))
                )
            ),
            alignTo(rob, Waypoints.swipe1, 0.7, 6), 
            alignTo(rob, Waypoints.postBumpIntake, 0.7, 6),
            alignTo(rob, Waypoints.preBump, 0.2, 6),
            new InstantCommand(rob.shooterHood::disableAutoDecap),
            new InstantCommand(() -> rob.turret.autoShoot = true),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    alignTo(rob, Waypoints.bumpToTrenchTransition1, 0.2, 0.5, 55),
                    alignTo(rob, Waypoints.bumpToTrenchTransition2, 0.2, 0.5, 55), // FIXME: make more transition points to make it better? big deadzone 
                    alignTo(rob, Waypoints.bumpToTrenchTransition3, 0.2, 0.5, 55),
                    alignTo(rob, Waypoints.startPos, 0.1, 0.7)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(1),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    )
                ),
                new RunSpindexer(rob)
            ),
            new InstantCommand(rob.shooterHood::enableAutoDecap),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    alignTo(rob, Waypoints.preSwipe2, 0.7, 6)
                ),
                new InstantCommand(() -> rob.intakePivot.setPosition(15.9)),
                new InstantCommand(() -> rob.intake.setVelocity(5000))
            ),
            alignTo(rob, Waypoints.swipe2, 0.1, 6),
            //alignTo(rob, Waypoints.posterBumpIntake, 0.3, 5),
            alignTo(rob, Waypoints.postBumpIntake_s2, 0.3, 3),
            alignTo(rob, Waypoints.postBump, 0.2, 6), // used to be preBump

            new InstantCommand(rob.shooterHood::disableAutoDecap),
            new InstantCommand(() -> rob.turret.autoShoot = true),
            new ParallelDeadlineGroup(
                // new SequentialCommandGroup(
                //     alignTo(rob, Waypoints.bumpToTrenchTransition1, 0.2, 0.5, 55),
                //     alignTo(rob, Waypoints.bumpToTrenchTransition2, 0.2, 0.5, 55), // FIXME: make more transition points to make it better? big deadzone 
                //     alignTo(rob, Waypoints.bumpToTrenchTransition3, 0.2, 0.5, 55),
                //     alignTo(rob, Waypoints.startPos, 0.1, 0.7)
                // ),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    ),
                    new WaitCommand(1),
                    new ParallelRaceGroup(
                        new AutoIntakeUnjam(rob),
                        new WaitCommand(0.5)
                    )
                ),
                new RunSpindexer(rob)
            ),
            new InstantCommand(rob.shooterHood::disableAutoDecap)
        );
    }

    public Command runAuto(RobotContainer rob, String autoToRun) {
        boolean opAuto = false;
        boolean doubleSwipe = false;
        boolean middle = false;
        switch (autoToRun) {
        case "blueleft":
            doubleSwipe = true;
            isAllianceFlipped = true;
            isRightFlipped = true;
            break;
        case "blueright":
            doubleSwipe = true;
            isAllianceFlipped = true;
            isRightFlipped = false;
            break;
        case "redright":
            doubleSwipe = true;
            isAllianceFlipped = false;
            isRightFlipped = true;
            break;
        case "redleft":
            doubleSwipe = true;
            isAllianceFlipped = false;
            isRightFlipped = false;
            break;
        case "opblueleft":
            opAuto = true;
            isAllianceFlipped = true;
            isRightFlipped = true;
            break;
        case "opblueright":
            opAuto = true;
            isAllianceFlipped = true;
            isRightFlipped = false;
            break;
        case "opredright":
            opAuto = true;
            isAllianceFlipped = false;
            isRightFlipped = true;
            break;
        case "opredleft":
            opAuto = true;
            isAllianceFlipped = false;
            isRightFlipped = false;
            break;
        case "blueleftbump":
            isAllianceFlipped = true;
            isRightFlipped = true;
            break;
        case "bluerightbump":
            isAllianceFlipped = true;
            isRightFlipped = false;
            break;
        case "redrightbump":
            isAllianceFlipped = false;
            isRightFlipped = true;
            break;
        case "redleftbump":
            isAllianceFlipped = false;
            isRightFlipped = false;
            break;
        case "redmiddle":
            middle = true;
            isAllianceFlipped = false;
            isRightFlipped = false;
            break;
        case "bluemiddle":
            middle = true;
            isRightFlipped = true;
            isAllianceFlipped = true;
            break;
        default:
            isAllianceFlipped = false;
            isRightFlipped = false;
            break;
        }

        buildAuto();
        if (autoToRun != "redmiddle" && autoToRun != "bluemiddle") {
            Pose2d startingPos;
            if (opAuto) {
                startingPos = Waypoints.startPosRotated;
            } else {
                startingPos = Waypoints.startPos;
            }
            if (isAllianceFlipped) startingPos = mirrorAlliance(startingPos);
            if (isRightFlipped) startingPos = mirrorToRight(startingPos);

            rob.drivetrain.resetPose(startingPos);
        } else {
            Pose2d startingPos;
            startingPos = Waypoints.middleStart;
            if (isAllianceFlipped) startingPos = mirrorAlliance(startingPos);
            if (isRightFlipped) startingPos = mirrorToRight(startingPos);
            rob.drivetrain.resetPose(startingPos);
        }

        SequentialCommandGroup auto = new SequentialCommandGroup();
        if (opAuto) {
            auto.addCommands(redLeftDoubleSwipeBump);
        } else if (doubleSwipe){
            auto.addCommands(redLeftDoubleSwipe);
        } else if (middle) {
            auto.addCommands(middleDepot);
        } else {
            auto.addCommands(redLeftDoubleSwipe1Bump);
        }

        return auto;
    }

    private Command alignTo(RobotContainer rob, Pose2d pose, double deadzone, double maxvel) {
        if (isRightFlipped) pose = mirrorToRight(pose);
        if (isAllianceFlipped) pose = mirrorAlliance(pose);
        if (isCloser && pose == Waypoints.swipe1) pose = Waypoints.swipe1Close;
        if (isCloser && pose == Waypoints.preSwipe1) pose = Waypoints.preSwipe1Close;
        if (!rob.isRed()) maxvel *= -1;

        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new DriveToPoint(rob, pose, deadzone),
                rob.drivetrain.driveToPoint(pose, maxvel)
            ),
            new InstantCommand(rob.drivetrain::stopSwerve)
        );
    }

    private Command alignTo(RobotContainer rob, Pose2d pose, double deadzone, double maxvel, double maxrotvel) {
        if (isRightFlipped) pose = mirrorToRight(pose);
        if (isAllianceFlipped) pose = mirrorAlliance(pose);
        if (isCloser && pose == Waypoints.swipe1) pose = Waypoints.swipe1Close;
        if (isCloser && pose == Waypoints.preSwipe1) pose = Waypoints.preSwipe1Close;
        if (!rob.isRed()) maxvel *= -1;
        maxrotvel = Math.toRadians(maxrotvel);

        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new DriveToPoint(rob, pose, deadzone),
                rob.drivetrain.driveToPoint(pose, maxvel, maxrotvel)
            ),
            new InstantCommand(rob.drivetrain::stopSwerve)
        );
    }
    
    private static Pose2d mirrorToRight(Pose2d pose) {
        return new Pose2d(pose.getX(), Constants.fieldYMeter - pose.getY(), Rotation2d.fromDegrees(-pose.getRotation().getDegrees()));
    }

    private static Pose2d mirrorAlliance(Pose2d pose){
        return new Pose2d(Constants.fieldXMeter - pose.getX(), pose.getY(), Rotation2d.fromDegrees(180 - pose.getRotation().getDegrees()));
    }
}
