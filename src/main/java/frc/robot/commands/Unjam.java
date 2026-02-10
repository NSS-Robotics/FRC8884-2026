package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Flywheels;

public class Unjam extends Command {
    private final Feeder feeder;
    private final Spindexer spindexer;
    private final Flywheels flywheels;

    public Unjam(RobotContainer rob) {
        feeder = rob.feeder;
        spindexer = rob.spindexer;
        flywheels = rob.flywheels;

        addRequirements(feeder, spindexer, flywheels);
    }

    @Override
    public void execute() {
        feeder.setSpeed(-5350);
        flywheels.setVelocity();
        spindexer.setSpeed(-100);
    }
}
