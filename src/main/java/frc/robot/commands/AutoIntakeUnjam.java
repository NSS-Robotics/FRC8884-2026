package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class AutoIntakeUnjam extends Command {
    private final Intake intake;
    private final IntakePivot intakePivot;

    public AutoIntakeUnjam(RobotContainer rob) {
        intake = rob.intake;
        intakePivot = rob.intakePivot;

        addRequirements(rob.intake, rob.intakePivot);
    }

    @Override
    public void execute() {
        // intake.setVelocity(2500); // can't run intake because it eats fabric
        intakePivot.setPosition(10); // used to be 8
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        intakePivot.setPosition(15.5);
    }
}
