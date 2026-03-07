package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Spindexer;

public class AutoRunSpindexer extends Command {
    private final Feeder feeder;
    private final Spindexer spindexer;

    public AutoRunSpindexer(RobotContainer rob) {
        feeder = rob.feeder;
        spindexer = rob.spindexer;
    }

    @Override
    public void execute() {
        spindexer.setSpeed(-2500);
        feeder.setSpeed(-2000);
    }
}
