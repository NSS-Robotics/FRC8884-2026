package frc.robot.commands;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.MotionMagicTurret;
import frc.robot.subsystems.ShooterHood;

public class FlywheelShoot extends Command {
    private final Flywheels flywheels;
    private final ShooterHood shooterHood;
    private final CommandSwerveDrivetrain swerve;


    private MotionMagicTurret mmturret;

    public FlywheelShoot(RobotContainer rob) {
        this.mmturret = rob.mmturret;
        this.flywheels = rob.flywheels;
        this.shooterHood = rob.shooterHood;
        this.swerve = rob.drivetrain;


        addRequirements(rob.flywheels);
    }

    @Override
    public void execute() {
        // flywheels.setVelocity(SmartDashboard.getNumber("Set RPM", 0));
        flywheels.setVelocity(flywheels.rpmToHub(mmturret.futureDistance(swerve.getPoseMeters(), mmturret.getDistanceToTarget(), Constants.targetPositionBlue))); //TODO: MAKE IT DEPEND ON RED OR BLUE
        // flywheels.setVelocity(mmturret.velocityToRpm(mmturret.ans[0], Constants.Flywheel.flywheelRadiusMeters));
        shooterHood.setDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
    }
}
