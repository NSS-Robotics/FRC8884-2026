package frc.robot.commands;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Feeder;

public class FlywheelShoot extends Command {

    private final RobotContainer rob;
    private final Flywheels flywheels;
    private final ShooterHood shooterHood;
    private final CommandSwerveDrivetrain swerve;
    private final Feeder feeder;

    private Turret turret;

    public FlywheelShoot(RobotContainer rob) {
        this.turret = rob.turret;
        this.flywheels = rob.flywheels;
        this.shooterHood = rob.shooterHood;
        this.swerve = rob.drivetrain;
        this.feeder = rob.feeder;
        this.rob = rob;

        addRequirements(rob.flywheels);
    }

    @Override
    public void execute() {

        // double future[] = turret.aLookIntoTheFuture(swerve.getPoseMeters(), turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        
        // double sotmV[] = turret.sotmPredictVelocity(swerve.getPoseMeters(), turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        // double sotmVA[] = turret.sotmPredictVelocityAcceleration(swerve.getPoseMeters(), turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        // double sotmVAI[] = turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), turret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);

        double sotmVI[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
                rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        double sotmVILobRight[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
                rob.isRed() ? Constants.rightCornerRed : Constants.rightCornerBlue);
        double sotmVILobLeft[] = rob.turret.sotmPredictVelocityIteration(swerve.getPoseMeters(), rob.turret.getDistanceToTarget(),
                rob.isRed() ? Constants.leftCornerRed : Constants.leftCornerBlue);
        
        // double future[] = turret.aLookIntoTheFuture(swerve.getPoseMeters(),
        // turret.getDistanceToTarget(), Constants.targetPositionBlue);

        // flywheels.setVelocity(SmartDashboard.getNumber("Set RPM", 0));

        // flywheels.setVelocity(flywheels.rpmToHub(sotmVI[0]));

        if (!turret.isInAllianceZone()) { 
            if (swerve.getPoseMeters().getY() > 4){
                flywheels.setVelocity(flywheels.rpmToHub(sotmVILobRight[0]));
            } else {
                flywheels.setVelocity(flywheels.rpmToHub(sotmVILobLeft[0]));
            } 
        } else {      
            // pointAtTarget(Constants.targetPositionBlue);
            flywheels.setVelocity(flywheels.rpmToHub(sotmVI[0]));
        }

        // flywheels.setVelocity(turret.velocityToRpm(turret.ans[0],
        // Constants.Flywheel.flywheelRadiusMeters));

        shooterHood.setDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
        feeder.stop();
    }
}
