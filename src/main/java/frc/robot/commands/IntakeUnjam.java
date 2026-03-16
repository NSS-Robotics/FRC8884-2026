package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
        intake.setVelocity(2500);
        intakePivot.setPosition(8);
        // Timer.delay(0.35);
        // intakePivot.setPosition(15.5);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        intakePivot.setPosition(15.5);
    }
}
