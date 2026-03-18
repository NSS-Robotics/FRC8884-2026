package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class IntakeUp extends Command {
    private final Intake intake;
    private final IntakePivot intakePivot;
    private final Turret turret;

    public IntakeUp(RobotContainer rob) {
        intake = rob.intake;
        intakePivot = rob.intakePivot;
        turret = rob.turret;
        addRequirements(rob.turret, rob.intake, rob.intakePivot);
    }

    @Override
    public void execute() {
       turret.isdocking = true;
       intakePivot.setPosition(5);
       Timer.delay(0.35);
       intakePivot.setPosition(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
