package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Spindexer;

public class RunSpindexer extends Command {
    private final Feeder feeder;
    private final Spindexer spindexer;

    public RunSpindexer(RobotContainer rob) {
        feeder = rob.feeder;
        spindexer = rob.spindexer;
    }

    @Override
    public void execute() {
        spindexer.setSpeed(-3000);
        // spindexer.makeSpin(-0.23);
        // feeder.makeSpin(-1.0/2.0);
        feeder.setSpeed(-500);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stop();
        feeder.stop();
    }
}