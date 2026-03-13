package frc.robot.commands;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.MotionMagicTurret;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Feeder;

public class FlywheelShoot extends Command {

    private final RobotContainer rob;
    private final Flywheels flywheels;
    private final ShooterHood shooterHood;
    private final CommandSwerveDrivetrain swerve;
    private final Feeder feeder;

    private MotionMagicTurret mmturret;

    public FlywheelShoot(RobotContainer rob) {
        this.mmturret = rob.mmturret;
        this.flywheels = rob.flywheels;
        this.shooterHood = rob.shooterHood;
        this.swerve = rob.drivetrain;
        this.feeder = rob.feeder;
        this.rob = rob;

        addRequirements(rob.flywheels);
    }

    @Override
    public void execute() {

        // double future[] = mmturret.aLookIntoTheFuture(swerve.getPoseMeters(), mmturret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        
        // double sotmV[] = mmturret.sotmPredictVelocity(swerve.getPoseMeters(), mmturret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        // double sotmVA[] = mmturret.sotmPredictVelocityAcceleration(swerve.getPoseMeters(), mmturret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        // double sotmVAI[] = mmturret.sotmPredictVelocityIteration(swerve.getPoseMeters(), mmturret.getDistanceToTarget(),
        //         rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        double sotmVI[] = mmturret.sotmPredictVelocityIteration(swerve.getPoseMeters(), mmturret.getDistanceToTarget(),
                rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue);
        
        // double future[] = mmturret.aLookIntoTheFuture(swerve.getPoseMeters(),
        // mmturret.getDistanceToTarget(), Constants.targetPositionBlue);

        // flywheels.setVelocity(SmartDashboard.getNumber("Set RPM", 0));

        flywheels.setVelocity(flywheels.rpmToHub(sotmVI[0]));


        // flywheels.setVelocity(mmturret.velocityToRpm(mmturret.ans[0],
        // Constants.Flywheel.flywheelRadiusMeters));

        shooterHood.setDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
        feeder.stop();
    }
}
