package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;

public class Limelight extends SubsystemBase {

    private NetworkTable table;
    private String name;

    public double ta = 0;
    public double tx = 0;
    public double ty = 0;
    public double tv = 0;

    public Pose2d botPose;
    public Pose3d botPoseTagRelative;

    public final CommandSwerveDrivetrain swerve;

    public Limelight(String name, CommandSwerveDrivetrain swerve) {
        this.name = "limelight-" + name;
        table = NetworkTableInstance.getDefault().getTable("limelight");

        this.swerve = swerve;
    }

    public void turnLimelightLED(boolean on) {
        table.getEntry("ledMode").setNumber(on ? 3 : 1);
    }

    public double getRotation() {
        return botPose.getRotation().getDegrees();
    }

    public void updateOdometry() {
        boolean useMegaTag2 = false;

        boolean shouldRejectUpdate = false;
        if (!useMegaTag2) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > 0.7 || mt1.rawFiducials[0].distToCamera > 3) {
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
            // worry about this later...
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

        if (pos.length < 6) {
            return;
        }

        String[] names = {
            "llpos x",
            "llpos y",
            "llpos z",
            "llrot x",
            "llrot y",
            "llrot z",
        };

        for (int i = 0; i < names.length; i++) {
            SmartDashboard.putNumber(name + ' ' + names[i], pos[i]);
        }
    }

    @Override
    public void periodic() {
        updateLimelightTracking();
        updateOdometry();

        SmartDashboard.putNumber("LL X", botPose.getX());
        SmartDashboard.putNumber("LL Y", botPose.getY());
        SmartDashboard.putNumber("LL R", botPose.getRotation().getDegrees());
    }
}
