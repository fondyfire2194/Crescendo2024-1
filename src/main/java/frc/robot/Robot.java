// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private double m_disableStartTime;

  private double brakeOffTime = 3;

  @Override
  public void robotInit() {

    DataLogManager.start();

    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();

  }

  @Override
  public void disabledPeriodic() {

    // if (m_robotContainer.m_af.checkChoiceChange())
    // m_robotContainer.m_af.validStartChoice =
    // m_robotContainer.m_af.selectAndLoadPathFiles();

    // turn off drive brakes if they are on and robotis not moving
    // allows easier manual pushing of robot

    if (m_robotContainer.m_swerve.driveIsBraked() && m_robotContainer.m_swerve.isStopped()) {
      if (m_disableStartTime == 0)
        m_disableStartTime = Timer.getFPGATimestamp();

      if (m_disableStartTime != 0 && Timer.getFPGATimestamp() > m_disableStartTime + brakeOffTime) {
        m_robotContainer.m_swerve.setIdleMode(false);
      }
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    m_robotContainer.m_swerve.setIdleMode(false);

    // m_autonomousCommand = m_robotContainer.m_af.getAutonomousCommand();

    SmartDashboard.putString("ACName", m_autonomousCommand.getName());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {

  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_swerve.setIdleMode(true);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
