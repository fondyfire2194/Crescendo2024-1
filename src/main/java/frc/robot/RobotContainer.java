// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionPhoton;

public class RobotContainer {
  /* Subsystems */
  final Swerve m_swerve = new Swerve();
  final VisionPhoton visPh = new VisionPhoton();
  final LimelightVision m_llv;

  private final Joystick driver = new Joystick(0);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  public RobotContainer() {

    String[] llnames = { "limelight", "limelight-1" };

    m_llv = new LimelightVision(llnames);
    
    setDefaultCommands();
    configureBindings();
  }

  private void configureBindings() {

  }

  private void setDefaultCommands() {

    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis) / 3,
            () -> robotCentric.getAsBoolean()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
