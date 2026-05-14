package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Feeder;

public class FlywheelShoot extends Command {
    private final RobotContainer rob;
    private final Flywheels flywheels;
    private final CommandSwerveDrivetrain swerve;
    private final Feeder feeder;

    private Turret turret;

    public FlywheelShoot(RobotContainer rob) {
        this.turret = rob.turret;
        this.flywheels = rob.flywheels;
        this.swerve = rob.drivetrain;
        this.feeder = rob.feeder;
        this.rob = rob;

        addRequirements(rob.flywheels);
    }

    @Override
    public void execute() {
        double sotmVI[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
                rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue, true);
        double sotmVILobRight[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
                rob.isRed() ? Constants.rightCornerRed : Constants.rightCornerBlue, true);
        double sotmVILobLeft[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
                rob.isRed() ? Constants.leftCornerRed : Constants.leftCornerBlue, true);
        
        // double future[] = turret.aLookIntoTheFuture(swerve.getPoseMeters(),
        // turret.getDistanceToTarget(), Constants.targetPositionBlue);

        rob.isShooting = true;

        if (rob.manualControl) {
            flywheels.setVelocity(SmartDashboard.getNumber("Set RPM", 0));
        } else {

            if (!turret.isInAllianceZone()) { 
                if (swerve.getPoseMeters().getY() > 4){
                    flywheels.setVelocity(flywheels.rpmToLob(sotmVILobRight[0]));
                } else {
                    flywheels.setVelocity(flywheels.rpmToLob(sotmVILobLeft[0]));
                } 
            } else {
                flywheels.setVelocity(flywheels.rpmToHub(sotmVI[0]));
            }
        }

        // flywheels.setVelocity(turret.velocityToRpm(turret.ans[0],
        // Constants.Flywheel.flywheelRadiusMeters));

        // shooterHood.setDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
        rob.isShooting = false;
    }
}
