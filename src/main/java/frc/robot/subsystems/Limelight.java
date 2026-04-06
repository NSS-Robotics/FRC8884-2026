package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {

    private NetworkTable table;
    private String name;

    public double ta = 0;
    public double tx = 0;
    public double ty = 0;
    public double tv = 0;

    public Pose2d botPose;
    public Pose3d botPoseTagRelative;

    private final IntegerLogEntry tagCountLog; 

    public final CommandSwerveDrivetrain swerve;

    public Limelight(RobotContainer rob, String name, CommandSwerveDrivetrain swerve) {
        this.name = "limelight-" + name;
        table = NetworkTableInstance.getDefault().getTable(this.name);

        this.swerve = swerve;

        tagCountLog = new IntegerLogEntry(rob.log, "/ll_"+name+"/tagcount");
    }

    public void turnLimelightLED(boolean on) {
        table.getEntry("ledMode").setNumber(on ? 3 : 1);
    }

    public double getRotation() {
        return botPose.getRotation().getDegrees();
    }

    public void updateOdometry() {
        boolean useMegaTag2 = true; // TODO: test this, if bad revert or try other stuff

        if (!useMegaTag2) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.name);

            // if limelight can't read any data, ignore
            if (mt1 == null) {
                return;
            }
            
            boolean shouldRejectUpdate = false;

            tagCountLog.append(mt1.tagCount);
            SmartDashboard.putNumber(this.name + " tagcount", mt1.tagCount);

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                boolean isTooFarForComfort = mt1.rawFiducials[0].distToCamera > 4;
                boolean isTooAmbiguiousForComfort = mt1.rawFiducials[0].ambiguity > 0.4;

                if (isTooAmbiguiousForComfort || isTooFarForComfort) {
                    shouldRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                shouldRejectUpdate = true;
            }

            if (!shouldRejectUpdate) {
                swerve.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        } else {
            LimelightHelpers.SetIMUMode(this.name, 4);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.name);

            // if limelight can't read any data, ignore
            if (mt2 == null) {
                return;
            }
            
            boolean shouldRejectUpdate = false;

            tagCountLog.append(mt2.tagCount);
            SmartDashboard.putNumber(this.name + " tagcount", mt2.tagCount);

            if (mt2.tagCount == 1 && mt2.rawFiducials.length == 1) {
                boolean isTooFarForComfort = mt2.rawFiducials[0].distToCamera > 4;
                boolean isTooAmbiguiousForComfort = mt2.rawFiducials[0].ambiguity > 0.4;

                if (isTooAmbiguiousForComfort || isTooFarForComfort) {
                    shouldRejectUpdate = true;
                }
            }
            if (mt2.tagCount == 0) {
                shouldRejectUpdate = true;
            }

            if (!shouldRejectUpdate) {
                swerve.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    public void updateLimelightTracking() {
        ta = table.getEntry("ta").getDouble(0);
        tx = table.getEntry("tx").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        tv = table.getEntry("tv").getDouble(0);

        double[] pos = table
            .getEntry("botpose_wpiblue")
            .getDoubleArray(new double[6]);

        SmartDashboard.putNumberArray("pos", pos);

        botPose = new Pose2d(
            pos[0], // X
            pos[1], // Y
            Rotation2d.fromDegrees(pos[5]) // R
        );

        // if (pos.length < 6) {
        //     return;
        // }

        // String[] names = {
        //     "llpos x",
        //     "llpos y",
        //     "llpos z",
        //     "llrot x",
        //     "llrot y",
        //     "llrot z",
        // };

        // for (int i = 0; i < names.length; i++) {
        //     SmartDashboard.putNumber(name + ' ' + names[i], pos[i]);
        // }
    }

    @Override
    public void periodic() {
        updateLimelightTracking();
        updateOdometry();

        SmartDashboard.putNumber(name + " X", botPose.getX());
        SmartDashboard.putNumber(name + " Y", botPose.getY());
        SmartDashboard.putNumber(name + " R", botPose.getRotation().getDegrees());
    }
}
