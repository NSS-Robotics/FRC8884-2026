package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private final Intake intake;

    public RunIntake(RobotContainer rob) {
        intake = rob.intake;

        // SmartDashboard.putNumber("Intake RPM", 0);
    }

    @Override
    public void execute() {
        // intake.setVelocity(SmartDashboard.getNumber("Intake RPM", 0));
        intake.setVelocity(3800);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();


        
    }
}
