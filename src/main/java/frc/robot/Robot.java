// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final Timer timer = new Timer();
  private Shift shift = Shift.TRANSITION;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.fieldMap.setRobotPose(m_robotContainer.drivetrain.getPoseMeters());
    // m_robotContainer.fieldMap.setRobotPose(new Pose2d(
      // m_robotContainer.simulatedPosition.getX(),
      // m_robotContainer.simulatedPosition.getY(),
      // Rotation2d.fromDegrees(m_robotContainer.turret.getDegrees())
    // ));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    timer.start();
  }

  @Override
  public void teleopPeriodic() {
    // double time = DriverStationJNI.getMatchTime();
    // double prephaseSwitch = 2; // seconds
    // //seconds? teleop is 140 seconds

    // if (time == 130 + prephaseSwitch){
    //   System.out.println("transition switch");
    // }
    // else if (time == 105 + prephaseSwitch){
    //   System.out.println("alliance switch");
    // }
    // else if (time == 80 + prephaseSwitch){
    //   System.out.println("alliance switch");
    // }
    // else if (time == 55 + prephaseSwitch){
    //   System.out.println("alliance switch");
    // }
    // else if (time == 30 + prephaseSwitch){
    //   System.out.println("endgame");
    // }
    SmartDashboard.putNumber("Timer", timer.get());
    //TODO: add rumble hehe
    if (timer.get() < 10)       shift = Shift.TRANSITION;
    else if (timer.get() < 35)  shift = Shift.ALLIANCEA;
    else if (timer.get() < 60)  shift = Shift.ALLIANCEB;
    else if (timer.get() < 85)  shift = Shift.ALLIANCEA;
    else if (timer.get() < 110) shift = Shift.ALLIANCEB;
    else                        shift = Shift.ENDGAME;

    SmartDashboard.putString("Shift", shift.toString());
  }

  @Override
  public void teleopExit() {
    timer.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
