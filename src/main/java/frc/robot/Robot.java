// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.PathFactory.amppaths;
import frc.robot.PathFactory.centerpaths;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private double m_disableStartTime;

  private double brakeOffTime = 3;

  private double m_startDelay;

  private double startTime;

  private boolean autoHasRun;

  private boolean firstScan = true;;

  @Override
  public void robotInit() {

    DataLogManager.start();

    m_robotContainer = new RobotContainer();

    // CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

    m_robotContainer.m_shooter.testjs = (-m_robotContainer.tstjs.getThrottle() + 1) / 2;
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    autoHasRun = false;

    m_robotContainer.m_pf.ampFilesOK = m_robotContainer.m_pf.checkAmpFilesExist();
    m_robotContainer.m_pf.centerFilesOK = m_robotContainer.m_pf.checkCenterFilesExist();
    m_robotContainer.m_pf.sourceFilesOK = m_robotContainer.m_pf.checkSourceFilesExist();

    Shuffleboard.selectTab("Autonomous");

  }

  @Override
  public void disabledPeriodic() {

    boolean checkAutos = firstScan || m_robotContainer.m_af.checkChoiceChange();

    firstScan = false;

    if (checkAutos) {
      m_robotContainer.m_af.validStartChoice = m_robotContainer.m_af.selectAndLoadPathFiles();

    }
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

    m_startDelay = m_robotContainer.m_af.m_startDelayChooser.getSelected();

    startTime = Timer.getFPGATimestamp();

    // m_autonomousCommand = m_robotContainer.m_cf.getAutonomousCommand();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (!autoHasRun && Timer.getFPGATimestamp() > startTime + m_startDelay
        && m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      autoHasRun = true;
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.m_shooter.stopMotors();
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
