package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class IntakeUnjam extends Command {
    private final Intake intake;
    private final IntakePivot intakePivot;

    public IntakeUnjam(RobotContainer rob) {
        intake = rob.intake;
        intakePivot = rob.intakePivot;

        addRequirements(rob.intake, rob.intakePivot);
    }

    @Override
    public void execute() {
        intake.setVelocity(3600);
        intakePivot.setPosition(8);
    }

    @Override
    public void end(boolean interrupted) {
        // intake.stopIntake();
        intake.setVelocity(4500);
        intakePivot.setPosition(15.5);
    }
}
