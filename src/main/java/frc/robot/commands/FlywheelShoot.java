package frc.robot.commands;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.MotionMagicTurret;
import frc.robot.subsystems.ShooterHood;

public class FlywheelShoot extends Command {
    private final Flywheels flywheels;
    private final ShooterHood shooterHood;

    private MotionMagicTurret mmturret;

    public FlywheelShoot(RobotContainer rob) {
        this.mmturret = rob.mmturret;
        this.flywheels = rob.flywheels;
        this.shooterHood = rob.shooterHood;

        addRequirements(rob.flywheels);
    }

    @Override
    public void execute() {
        // flywheels.setVelocity(SmartDashboard.getNumber("Set RPM", 0));
        flywheels.setVelocity(flywheels.rpmToHub(mmturret.getDistanceToTarget()));
        shooterHood.setDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
    }
}
