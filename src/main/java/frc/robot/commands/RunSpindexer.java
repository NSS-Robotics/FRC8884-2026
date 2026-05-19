package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Spindexer;

public class RunSpindexer extends Command {
    private final Feeder feeder;
    private final Spindexer spindexer;

    private final RobotContainer rob;

    public RunSpindexer(RobotContainer rob) {
        feeder = rob.feeder;
        spindexer = rob.spindexer;
        this.rob = rob;
        addRequirements(rob.spindexer, rob.feeder);
    }

    @Override
    public void execute() {
        double spindexerSpeed = -5000;
        if (rob.manualControl) {
            SmartDashboard.getNumber("Set Spindexer Speed", -5000);
        }
        spindexer.setSpeed(spindexerSpeed);
        // spindexer.makeSpin(-0.23);
        // feeder.makeSpin(-1.0/2.0);
        feeder.setSpeed(-5500);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stop();
        feeder.stop();
    }
}