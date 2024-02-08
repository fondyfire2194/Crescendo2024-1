// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class DriveToGamePiece extends Command {

  private LimelightSubsystem ll;
  private SwerveSubsystem drivetrain;
  private PIDController thetaController = new PIDController(4.0, 0, 0.05);

  public DriveToGamePiece(SwerveSubsystem drivetrain, LimelightSubsystem ll) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.ll = ll;
  }

  private double thetaOutput = 0;
  private final double xOutput = 0.2; // Speed to drive towards note will increase after testing
  private final double yOutput = 0;
  private double setpoint = 0; // How far the camera is offset from the center in degrees

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ll.hasTarget()) {
      setpoint = Math.toRadians(-ll.getTX()) + drivetrain.getPose().getRotation().getRadians();
      thetaController.setSetpoint(setpoint);
      if (!thetaController.atSetpoint()) {
        thetaOutput = thetaController.calculate(drivetrain.getPose().getRotation().getRadians(), setpoint);
      }
    }

    drivetrain.drive(-xOutput * SwerveConstants.maxSpeed, yOutput * SwerveConstants.maxSpeed,
        thetaOutput * SwerveConstants.maxAngularVelocity, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}