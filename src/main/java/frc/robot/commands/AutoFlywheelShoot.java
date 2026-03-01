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

public class AutoFlywheelShoot extends Command {
    
    private final RobotContainer rob;
    private final Flywheels flywheels;
    private final ShooterHood shooterHood;
    private final CommandSwerveDrivetrain swerve;


    private MotionMagicTurret mmturret;

    public AutoFlywheelShoot(RobotContainer rob) {
        this.mmturret = rob.mmturret;
        this.flywheels = rob.flywheels;
        this.shooterHood = rob.shooterHood;
        this.swerve = rob.drivetrain;
        this.rob = rob;


        addRequirements(rob.flywheels);
    }

    @Override
    public void execute() {

        double future[] = mmturret.aLookIntoTheFuture(swerve.getPoseMeters(), mmturret.getDistanceToTarget(), rob.isRed() ? Constants.targetPositionRed : Constants.targetPositionBlue); 
        // double future[] = mmturret.aLookIntoTheFuture(swerve.getPoseMeters(), mmturret.getDistanceToTarget(), Constants.targetPositionBlue); 
        // flywheels.setVelocity(SmartDashboard.getNumber("Set RPM", 0));
        flywheels.setVelocity(flywheels.rpmToHub(future[0]));   
        // flywheels.setVelocity(mmturret.velocityToRpm(mmturret.ans[0], Constants.Flywheel.flywheelRadiusMeters));
        shooterHood.setDegrees();
    }

}
