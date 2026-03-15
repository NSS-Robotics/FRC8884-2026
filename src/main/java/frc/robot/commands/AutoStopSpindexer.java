package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Spindexer;

public class AutoStopSpindexer extends Command {
    private final Feeder feeder;
    private final Spindexer spindexer;

    public AutoStopSpindexer(RobotContainer rob) {
        feeder = rob.feeder;
        spindexer = rob.spindexer;
    }

    @Override
    public void execute() {
        spindexer.isAutoShoot = false;
        feeder.isAutoShoot = false;
        spindexer.stop();
        feeder.stop();
    }
    @Override
    public void end(boolean interrupted) {

    }
}
