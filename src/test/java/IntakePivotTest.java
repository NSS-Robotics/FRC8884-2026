import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakePivot;

public class IntakePivotTest {
    private RobotContainer robotContainer;
    private IntakePivot intakePivot;

    @BeforeEach
    private void setup() {
        robotContainer = new RobotContainer();
        intakePivot = robotContainer.intakePivot;
    }

    @Test
    private void goesDown() {
        intakePivot.setPosition(15.9);
        assertEquals(15.9, intakePivot.getPosition());
    }

    @Test
    private void knowsWhenUp() {
        // Test that the intake pivot knows when it's raised
        intakePivot.setPosition(0.1);
        assertEquals(true, intakePivot.isIntakeUp());
        intakePivot.setPosition(1.0);
        assertEquals(true, intakePivot.isIntakeUp());
        intakePivot.setPosition(0.1);
        assertEquals(true, intakePivot.isIntakeUp());
        intakePivot.setPosition(0.1);
        assertEquals(true, intakePivot.isIntakeUp());
    }
}
