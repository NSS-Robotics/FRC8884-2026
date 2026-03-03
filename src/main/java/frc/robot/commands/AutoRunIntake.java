package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class AutoRunIntake extends Command {
    private final Intake intake;

    public AutoRunIntake(RobotContainer rob) {
        intake = rob.intake;
    }

    @Override
    public void execute() {
        intake.setVelocity(4000);
    }
}
