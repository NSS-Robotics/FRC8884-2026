package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private final Intake intake;

    public RunIntake(RobotContainer rob) {
        intake = rob.intake;
    }

    @Override
    public void execute() {
        intake.setVelocity(4000);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
